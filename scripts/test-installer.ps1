param([Parameter(Mandatory=$true)][string]$PackageDirectory)
$ErrorActionPreference='Stop'
$package=(Resolve-Path -LiteralPath $PackageDirectory).Path
$root=Split-Path -Parent $PSScriptRoot
$run=Join-Path $root ('work\installer-tests-'+[guid]::NewGuid().ToString('N'))
$game=Join-Path $run 'fake-game'
New-Item -ItemType Directory -Path $game -Force|Out-Null
function Put([string]$Relative,[string]$Value){$path=Join-Path $game $Relative;New-Item -ItemType Directory -Path (Split-Path -Parent $path) -Force|Out-Null;[IO.File]::WriteAllText($path,$Value)}
function Require([bool]$Value,[string]$Message){if(!$Value){throw $Message}}
function Snapshot {
 $files=[ordered]@{};$backups=Join-Path $game 'BepInEx\BetterPenetrationNextBackups'
 foreach($f in Get-ChildItem -LiteralPath $game -Recurse -File|Sort-Object FullName){if(!$f.FullName.StartsWith($backups+'\',[StringComparison]::OrdinalIgnoreCase)){$files[$f.FullName.Substring($game.Length+1)]=(Get-FileHash -LiteralPath $f.FullName).Hash}}
 return ($files|ConvertTo-Json -Compress)
}
# Transaction fixture only. These files are never executed or loaded as plugins.
foreach($relative in @('KoikatsuSunshine.exe','BepInEx\core\BepInEx.dll','BepInEx\core\0Harmony.dll','BepInEx\plugins\KKSAPI.dll','BepInEx\plugins\ModBoneImplantor.dll','BepInEx\plugins\KKS_UncensorSelector.dll','BepInEx\plugins\KKS_Sideloader.dll','BepInEx\plugins\KKS_ExtensibleSaveFormat.dll')){Put $relative ('fixture '+$relative)}
Put 'BepInEx\plugins\BetterPenetration\KKS_BetterPenetration.dll' 'original controller'
Put 'mods\BetterPenetration\BetterPenetrationNext-1.0.zipmod' 'original receiver'
Put 'mods\BetterPenetration\[KKS][Penis][BP] SoS.zipmod' 'original source unchanged'
Put 'BepInEx\config\org.betterpenetration.next.cfg' "[Next.Contact]`nContactDamping = 3`n[Other]`nKeep = user setting`n"
$before=Snapshot
& (Join-Path $package 'Install.ps1') -GameRoot $game|Out-Null
$backupRoot=Join-Path $game 'BepInEx\BetterPenetrationNextBackups'
$first=(Get-ChildItem -LiteralPath $backupRoot -Directory|Sort-Object LastWriteTimeUtc|Select-Object -Last 1).FullName
Require (!(Test-Path -LiteralPath (Join-Path $game 'BepInEx\plugins\BetterPenetration\KKS_BetterPenetration.dll'))) 'Original BP remains active'
Require (!(Test-Path -LiteralPath (Join-Path $game 'mods\BetterPenetration\BetterPenetrationNext-1.0.zipmod'))) 'Original receiver remains active'
$cfg=Join-Path $game 'BepInEx\config\org.betterpenetration.next.cfg';$text=[IO.File]::ReadAllText($cfg)
$profile=Get-Content -LiteralPath (Join-Path $package 'release-defaults.json') -Raw|ConvertFrom-Json
$damping=($profile.Settings|Where-Object {$_.Section -eq 'Next.Contact' -and $_.Key -eq 'ContactDamping'}).Value
Require ($text -match 'Keep = user setting' -and $text -match 'ToggleXRay = B' -and $text -match 'OpenSettings = N' -and $text.Contains('ContactDamping = '+$damping)) 'Installed defaults/shortcuts or unrelated values differ'
$installed=Snapshot
Write-Output 'PASS fresh install disables active legacy files and preserves unrelated data'
# This is the already-disabled-original state of previous BPN test builds.
[IO.File]::WriteAllText($cfg,$text.Replace(('ContactDamping = '+$damping),'ContactDamping = 2'))
$edited=Snapshot
& (Join-Path $package 'Install.ps1') -GameRoot $game|Out-Null
$second=(Get-ChildItem -LiteralPath $backupRoot -Directory|Where-Object FullName -ne $first|Select-Object -First 1).FullName
Require ((Snapshot) -ceq $installed) 'Upgrade did not reapply the tested defaults'
& (Join-Path $second 'Rollback.ps1') -BackupDirectory $second|Out-Null
Require ((Snapshot) -ceq $edited) 'Upgrade rollback did not restore user-edited previous configuration byte-for-byte'
Write-Output 'PASS upgrade with disabled original and byte-exact rollback'
& (Join-Path $first 'Rollback.ps1') -BackupDirectory $first|Out-Null
Require ((Snapshot) -ceq $before) 'Fresh rollback did not restore all original files byte-for-byte'
Write-Output 'PASS fresh rollback restores original DLL, receiver, config and unchanged Source'
$broken=Join-Path $run 'corrupt-package';Copy-Item -LiteralPath $package -Destination $broken -Recurse
[IO.File]::AppendAllText((Join-Path $broken 'release-defaults.json'),"`n")
$rejected=$false;try{& (Join-Path $broken 'Install.ps1') -GameRoot $game|Out-Null}catch{$rejected=$true}
Require $rejected 'Tampered package was accepted';Require ((Snapshot) -ceq $before) 'Tamper rejection changed the game fixture'
Write-Output 'PASS tampered package rejected before any target-file change'
[ordered]@{Passed=$true;Scope='Filesystem transactions on inert game fixture; not Unity dependency compatibility';Fresh=$true;Upgrade=$true;Rollback=$true;TamperRejected=$true}|ConvertTo-Json|Set-Content -LiteralPath (Join-Path $run 'result.json') -Encoding UTF8
Write-Output ('Evidence: '+$run)
