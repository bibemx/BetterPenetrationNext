function Assert-TargetGameClosed([string]$GameDirectory) {
    $gamePath=[IO.Path]::GetFullPath($GameDirectory).TrimEnd('\')
    $targets=@((Join-Path $gamePath 'KoikatsuSunshine.exe'),(Join-Path $gamePath 'CharaStudio.exe'))
    $running=@(Get-Process -Name KoikatsuSunshine,CharaStudio -ErrorAction SilentlyContinue)
    foreach($process in $running) {
        $executable=$process.Path
        if([string]::IsNullOrEmpty($executable)) { throw 'Cannot verify a running KKS/Studio path. Close it or run with permission to read process details.' }
        if($executable -in $targets) { throw 'Close KKS and Studio from this installation before changing files.' }
    }
}
