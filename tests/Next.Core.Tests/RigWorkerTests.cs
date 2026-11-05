using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;
using BetterPenetrationNext.Core;

internal static class RigWorkerTests
{
    static void Require(bool value,string message){if(!value)throw new Exception(message);}
    static void Wait(Func<bool> condition,string message){var timer=Stopwatch.StartNew();while(!condition()){if(timer.ElapsedMilliseconds>3000)throw new Exception(message);Thread.Sleep(1);}}
    static ReactiveRigInput Input(){return new ReactiveRigInput(1,1,3){Frame=7,Dt=.025,SourceRoot=new Vec3(1,2,3)};}
    static ReactiveRigOutput Output(){return new ReactiveRigOutput(4,2,3,2);}

    public static void Run(Action<string,Action> test)
    {
        test("Worker rejects an invalid landmark count before changing an unread snapshot",()=>{
            using(var worker=new ReactiveRigWorker((input,output)=>{
                output.Frame=input.Frame;output.Offsets[0]=new Vec3(input.Frame,0,0);output.ReceiverLandmarkCount=input.Frame==7?1:int.MaxValue;
                output.ReceiverLandmarkPoints[0]=new Vec3(0,input.Frame,0);
            },2,1,1,4,2,3)){
                var frame=Input();Require(worker.Submit(frame),"Initial submission rejected");Wait(()=>!worker.Busy,"Initial worker did not finish");
                frame.Frame=8;Require(worker.Submit(frame),"Invalid-output test submission rejected");Wait(()=>!worker.Busy,"Invalid output left worker busy");Require(worker.Failure!=null,"Invalid output was not reported");
                var result=Output();Require(worker.TryRead(result),"Invalid output destroyed the previous unread result");
                Require(result.Frame==7&&result.Offsets[0].X==7&&result.ReceiverLandmarkCount==1&&result.ReceiverLandmarkPoints[0].Y==7,"Invalid output partially overwrote the previous snapshot");
            }
        });
        test("Worker rejects negative diagnostic counts and can publish a later valid snapshot",()=>{
            using(var worker=new ReactiveRigWorker((input,output)=>{output.Frame=input.Frame;output.ReceiverLandmarkCount=input.Frame==7?-1:1;output.ReceiverLandmarkPoints[0]=new Vec3(input.Frame,0,0);},2,1,1,4,2,3)){
                var frame=Input();Require(worker.Submit(frame),"Submission rejected");Wait(()=>!worker.Busy,"Negative diagnostic count left worker busy");var result=Output();Require(worker.Failure!=null&&!worker.TryRead(result),"Invalid snapshot became available");
                frame.Frame=8;Require(worker.Submit(frame),"Worker did not accept recovery submission");Wait(()=>worker.TryRead(result),"Valid snapshot after a failed publication missing");Require(result.Frame==8&&result.ReceiverLandmarkCount==1&&result.ReceiverLandmarkPoints[0].X==8,"Recovery snapshot corrupt");
            }
        });
        test("Worker preserves the real terminal frame and contact-induced canal extension",()=>{
            using(var worker=new ReactiveRigWorker((input,output)=>{output.Frame=input.Frame;output.SourceGuidedCentres[7]=new Vec3(7,8,input.Frame);output.CervicalMotionAxis=new Vec3(0,0,1);output.GuidanceCervicalDisplacement=new Vec3(0,0,.012);output.TerminalAvailable=true;output.TerminalCentre=new Vec3(1,2,input.Frame);output.TerminalNormal=new Vec3(0,1,0);output.TerminalRadius=.002;output.TerminalTipAxial=.003;output.TerminalTipLateral=.004;output.ContactCanalExtension=.02;output.TerminalPoints[4]=new Vec3(4,5,input.Frame);},2,1,1,4,2,3))
            {
                var frame=Input();var first=Output();Require(worker.Submit(frame),"Submit rejected");Wait(()=>worker.TryRead(first),"Missing terminal snapshot");
                Require(first.TerminalAvailable&&first.TerminalCentre.Z==7&&first.TerminalNormal.Y==1&&first.TerminalPoints[4].Z==7,"Terminal landmark was lost across the worker boundary");
                Require(first.TerminalRadius==.002&&first.TerminalTipAxial==.003&&first.TerminalTipLateral==.004&&first.ContactCanalExtension==.02,"Terminal metrics did not describe the published geometry");
                Require(first.CervicalMotionAxis.Z==1&&first.GuidanceCervicalDisplacement.Z==.012&&first.SourceGuidedCentres[7].Z==7,"Guide/contact separation was lost across the worker boundary");
                first.TerminalPoints[4]=new Vec3(99,99,99);frame.Frame=8;Require(worker.Submit(frame),"Second submit rejected");var second=Output();Wait(()=>worker.TryRead(second),"Missing next terminal snapshot");Require(second.TerminalPoints[4].Z==8,"Terminal points alias the consumer buffer");
            }
        });
        test("Worker snapshots input arrays and rejects an unbounded queue",()=>{
            using(var entered=new ManualResetEventSlim())using(var proceed=new ManualResetEventSlim()){
                int caller=Thread.CurrentThread.ManagedThreadId;var frame=Input();frame.Source[0]=new Vec3(1,2,3);frame.Morph[0]=.4;frame.Translation[0]=new Vec3(4,5,6);
                using(var worker=new ReactiveRigWorker((input,output)=>{
                    entered.Set();Require(proceed.Wait(3000),"Test release timeout");output.Frame=input.Frame;output.WorkerThread=Thread.CurrentThread.ManagedThreadId;
                    output.SourceWorld[0]=input.Source[0];output.Offsets[0]=input.Translation[0];output.RecoveryFraction=input.Morph[0];output.TotalMs=input.Dt;
                },2,1,1,4,2,3)){
                    try{
                        Require(worker.Submit(frame)&&entered.Wait(3000),"Worker did not start");
                        frame.Source[0]=new Vec3(99,99,99);frame.Translation[0]=new Vec3(99,99,99);frame.Morph[0]=9;frame.Frame=99;frame.Dt=9;
                        for(int i=0;i<100;i++)Require(!worker.Submit(frame),"Outstanding work was overwritten");
                        Require(worker.Dropped==100&&worker.Submitted==1,"Work queue was not bounded");
                    }finally{proceed.Set();}
                    var result=Output();Wait(()=>worker.TryRead(result),"Missing worker snapshot");
                    Require(result.Frame==7&&result.SourceWorld[0].X==1&&result.Offsets[0].X==4&&result.RecoveryFraction==.4&&result.TotalMs==.025,"Input snapshot changed after submit");
                    Require(result.WorkerThread!=caller,"Numerical work ran on the caller thread");Require(!worker.TryRead(result),"Snapshot was consumed twice");
                }
            }
        });
        test("Worker transfers constraint diagnostics and keeps published geometry independent",()=>{
            using(var worker=new ReactiveRigWorker((input,output)=>{output.Frame=input.Frame;output.LimitingEdge=42;output.SourceOffsets[1]=new Vec3(input.Frame,2,3);output.LandmarkCount=2;output.LandmarkPoints[1]=new Vec3(4,5,input.Frame);},2,1,1,4,2,3)){
                var frame=Input();var first=Output();Require(worker.Submit(frame),"Submit rejected");Wait(()=>worker.TryRead(first),"Missing first snapshot");
                Require(first.LimitingEdge==42&&first.LandmarkCount==2&&first.LandmarkPoints[1].Z==7,"Constraint or landmark diagnostics were lost");
                first.SourceOffsets[1]=new Vec3(100,100,100);frame.Frame=8;Require(worker.Submit(frame),"Second submit rejected");var second=Output();Wait(()=>worker.TryRead(second),"Missing second snapshot");
                Require(second.SourceOffsets[1].X==8&&second.LandmarkPoints[1].Z==8,"Published geometry aliases a consumer array");
            }
        });
        test("Worker reports processor failures and accepts a later valid frame",()=>{
            int calls=0;using(var worker=new ReactiveRigWorker((input,output)=>{if(Interlocked.Increment(ref calls)==1)throw new InvalidOperationException("neutral test failure");output.Frame=input.Frame;},2,1,1,4,2,3)){
                var frame=Input();Require(worker.Submit(frame),"Submit rejected");Wait(()=>!worker.Busy,"Failed processor remained busy");Require(worker.Failure!=null&&worker.Failure.Contains("neutral test failure"),"Failure was suppressed");
                frame.Frame=8;Require(worker.Submit(frame),"Failure permanently stopped processing");var result=Output();Wait(()=>worker.TryRead(result),"Missing recovered result");Require(result.Frame==8,"Wrong recovered frame");
            }
        });
        test("Worker dimension errors preserve an unread result",()=>{
            using(var worker=new ReactiveRigWorker((input,output)=>output.Frame=input.Frame,2,1,1,4,2,3)){
                bool rejected=false;try{worker.Submit(new ReactiveRigInput(0,0,0));}catch(ArgumentException){rejected=true;}Require(rejected&&!worker.Busy,"Invalid dimensions partially submitted work");
                Require(worker.Submit(Input()),"Valid submit rejected");Wait(()=>!worker.Busy,"Worker remained busy");
                rejected=false;try{worker.TryRead(new ReactiveRigOutput(1,1,1,1));}catch(ArgumentException){rejected=true;}Require(rejected,"Mismatched result dimensions accepted");
                var result=Output();Require(worker.TryRead(result)&&result.Frame==7,"Rejected read consumed a valid result");
            }
        });
        test("Worker shutdown is idempotent and safe during concurrent submission",()=>{
            for(int cycle=0;cycle<64;cycle++){
                var worker=new ReactiveRigWorker((input,output)=>output.Frame=input.Frame,2,1,1,4,2,3);var frame=Input();
                var submit=Task.Run(()=>{for(int i=0;i<100;i++)worker.Submit(frame);});var stop=Task.Run(()=>worker.Dispose());Task.WaitAll(submit,stop);
                worker.Dispose();Require(!worker.Submit(frame),"Disposed worker accepted work");
            }
        });
        test("Worker hot processing and snapshot exchange reuse their managed buffers",()=>{
            long allocated=0;int calls=0;using(var worker=new ReactiveRigWorker((input,output)=>{
                long before=GC.GetAllocatedBytesForCurrentThread();for(int i=0;i<input.Source.Length;i++)output.SourceWorld[i]=input.Source[i]+input.SourceRoot;output.Frame=input.Frame;
                long bytes=GC.GetAllocatedBytesForCurrentThread()-before;if(++calls>8)Interlocked.Add(ref allocated,bytes);
            },2,1,1,4,2,3)){
                var frame=Input();var result=Output();for(int i=0;i<100;i++){frame.Frame=i;Wait(()=>worker.Submit(frame),"Worker did not become ready");Wait(()=>worker.TryRead(result),"Missing hot result");Require(result.Frame==i,"Stale result accepted");}
                Require(Interlocked.Read(ref allocated)==0,"Numerical processor allocated in its hot loop");
            }
        });
        test("Diagnostic buffers scale with the number of material sections",()=>{
            var output=new ReactiveRigOutput(4,2,3,80);Require(output.LandmarkPoints.Length>=2+33*17+80*5,"Landmark buffer assumes only seventeen receiver sections");
        });
    }
}
