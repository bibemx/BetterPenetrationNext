param([Parameter(Mandatory=$true)][string]$GameRoot)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
. (Join-Path $root 'release\ReleaseDefaults.ps1')
$profilePath=Join-Path $root 'profiles\release-defaults.json'
$profile=Read-ReleaseDefaults $profilePath
$run=Join-Path $root ('work\defaults-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $run -Force|Out-Null
Write-ReleaseDefaults (Join-Path $run 'installer.cfg') $profile $run
& (Join-Path $PSScriptRoot 'build.ps1') -GameRoot $GameRoot
dotnet run --project (Join-Path $root 'tests\ConfigDefaults') -c Release ('-p:GameRoot='+$GameRoot) -- $profilePath $run
if($LASTEXITCODE){throw 'Defaults tests failed'}
