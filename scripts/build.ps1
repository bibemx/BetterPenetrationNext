param([Parameter(Mandatory=$true)][string]$GameRoot)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
dotnet build (Join-Path $root 'src\Next.Plugin\Next.Plugin.csproj') -c Release ('-p:GameRoot='+$GameRoot) -m:1
if($LASTEXITCODE){throw 'Plugin build failed'}
