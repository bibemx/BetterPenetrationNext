$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
dotnet run --project (Join-Path $root 'tests\Next.Core.Tests') -c Release
if($LASTEXITCODE){throw 'Core tests failed'}
