param([string]$GameRoot)
$ErrorActionPreference='Stop'
if(!$GameRoot){$GameRoot=Read-Host 'KKS directory (contains KoikatsuSunshine.exe)'}
$game=(Resolve-Path -LiteralPath $GameRoot.Trim('"')).Path
if(!(Test-Path -LiteralPath (Join-Path $game 'KoikatsuSunshine.exe'))){throw 'KoikatsuSunshine.exe not found'}
. (Join-Path $PSScriptRoot 'GameGuard.ps1')
. (Join-Path $PSScriptRoot 'ReleaseDefaults.ps1')
Assert-TargetGameClosed $game
$package=[IO.Path]::GetFullPath($PSScriptRoot).TrimEnd('\')
$manifest=Get-Content -LiteralPath (Join-Path $package 'package.json') -Raw|ConvertFrom-Json
foreach($entry in $manifest.Files){$file=[IO.Path]::GetFullPath((Join-Path $package $entry.Path));if(!$file.StartsWith($package+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Invalid package path'};if((Get-FileHash -LiteralPath $file).Hash -ne $entry.SHA256){throw ('Package integrity failed: '+$entry.Path)}}
foreach($relative in @('BepInEx\core\BepInEx.dll','BepInEx\core\0Harmony.dll')){if(!(Test-Path -LiteralPath (Join-Path $game $relative))){throw ('Missing dependency: '+$relative)}}
$plugins=Join-Path $game 'BepInEx\plugins'
$dlls=@(Get-ChildItem -LiteralPath $plugins -Recurse -File -Filter '*.dll')
foreach($required in @('KKSAPI.dll','ModBoneImplantor.dll','KKS_UncensorSelector.dll','KKS_Sideloader.dll','KKS_ExtensibleSaveFormat.dll')){if(!@($dlls|Where-Object Name -eq $required).Count){throw ('Missing dependency: '+$required)}}
$profilePath=Join-Path $package 'release-defaults.json';$profile=Read-ReleaseDefaults $profilePath
if($profile.Version -ne '1.0.9' -or $profile.Status -ne 'validated'){throw 'Wrong release defaults'}
$sourceDll=Join-Path $package 'BepInEx\plugins\BetterPenetrationNext.dll'
if((Get-EmbeddedDefaultsHash $sourceDll) -ne (Get-FileHash -LiteralPath $profilePath).Hash){throw 'DLL/defaults mismatch'}
$writes=@('BepInEx\plugins\BetterPenetrationNext.dll','BepInEx\config\org.betterpenetration.next.cfg','mods\BetterPenetration\BPN.Reciever.Profundis.zipmod')
$disable=@($dlls|Where-Object Name -Match '^(KKS_BetterPenetration|BetterPenetration|KKS_BetterPenetrationEnhanced)\.dll$'|ForEach-Object FullName)
$mods=Join-Path $game 'mods'
if(Test-Path -LiteralPath $mods){$disable+=@(Get-ChildItem -LiteralPath $mods -Recurse -File -Filter 'BetterPenetrationNext-1.0.zipmod'|ForEach-Object FullName)}
$pairs=@();foreach($path in $disable){$relative=$path.Substring($game.Length+1);$pairs+=@{Original=$relative;Disabled=($relative+'.bpn-disabled-'+[guid]::NewGuid().ToString('N'))}}
$targets=@($writes)+@($pairs|ForEach-Object {$_.Original;$_.Disabled})
$backup=Join-Path $game ('BepInEx\BetterPenetrationNextBackups\1.0.9-'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $backup -Force|Out-Null
$entries=@();foreach($relative in $targets){$path=[IO.Path]::GetFullPath((Join-Path $game $relative));if(!$path.StartsWith($game+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Install path leaves the game'};$had=Test-Path -LiteralPath $path;$file=[guid]::NewGuid().ToString('N')+'.backup';$hash=$null;if($had){$hash=(Get-FileHash -LiteralPath $path).Hash;Copy-Item -LiteralPath $path -Destination (Join-Path $backup $file);if((Get-FileHash -LiteralPath (Join-Path $backup $file)).Hash -ne $hash){throw 'Backup verification failed'}};$entries+=@{RelativePath=$relative;HadFile=$had;BackupFile=$file;SHA256=$hash}}
$record=@{Version='1.0.9';GameRoot=$game;Entries=$entries;DisabledPairs=$pairs;Status='prepared';Utc=[datetime]::UtcNow.ToString('o')}
Copy-Item -LiteralPath (Join-Path $package 'Rollback.ps1'),(Join-Path $package 'GameGuard.ps1') -Destination $backup
$record|ConvertTo-Json -Depth 8|Set-Content -LiteralPath (Join-Path $backup 'installation.json') -Encoding UTF8
try{
 Assert-TargetGameClosed $game
 foreach($pair in $pairs){Move-Item -LiteralPath (Join-Path $game $pair.Original) -Destination (Join-Path $game $pair.Disabled)}
 foreach($relative in @($writes[0],$writes[2])){$target=Join-Path $game $relative;New-Item -ItemType Directory -Path (Split-Path -Parent $target) -Force|Out-Null;Copy-Item -LiteralPath (Join-Path $package $relative) -Destination $target -Force;if((Get-FileHash -LiteralPath $target).Hash -ne (Get-FileHash -LiteralPath (Join-Path $package $relative)).Hash){throw 'Installed file differs from package'}}
 Write-ReleaseDefaults (Join-Path $game $writes[1]) $profile $game
 # KeyboardShortcut converters are registered by BepInEx inside Unity. Keep
 # these UI defaults outside the numerical profile; they also match Menu.cs.
 $controls=[pscustomobject]@{Settings=@(
  [pscustomobject]@{Section='Next.Controls';Key='Mode';Value='BPN'},
  [pscustomobject]@{Section='Next.Controls';Key='ToggleXRay';Value='B'},
  [pscustomobject]@{Section='Next.Controls';Key='OpenSettings';Value='N'})}
 Write-ReleaseDefaults (Join-Path $game $writes[1]) $controls $game
 $record.Status='installed';$record|ConvertTo-Json -Depth 8|Set-Content -LiteralPath (Join-Path $backup 'installation.json') -Encoding UTF8
 Write-Output ('Installed BetterPenetrationNext 1.0.9. Backup: '+$backup)
 Write-Output 'Original BP code is available through the BP mode. Source SoS must be installed separately.'
}catch{& (Join-Path $backup 'Rollback.ps1') -BackupDirectory $backup;throw}
