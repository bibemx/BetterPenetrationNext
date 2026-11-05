param([string]$BackupDirectory)
$ErrorActionPreference='Stop'
if(!$BackupDirectory){$BackupDirectory=$PSScriptRoot}
$backup=(Resolve-Path -LiteralPath $BackupDirectory.Trim('"')).Path
$record=Get-Content -LiteralPath (Join-Path $backup 'installation.json') -Raw|ConvertFrom-Json
$game=(Resolve-Path -LiteralPath $record.GameRoot).Path
$allowed=Join-Path $game 'BepInEx\BetterPenetrationNextBackups'
if(!$backup.StartsWith($allowed+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Backup is outside the game backup directory'}
. (Join-Path $PSScriptRoot 'GameGuard.ps1');Assert-TargetGameClosed $game
foreach($entry in $record.Entries){$target=[IO.Path]::GetFullPath((Join-Path $game $entry.RelativePath));if(!$target.StartsWith($game+'\',[StringComparison]::OrdinalIgnoreCase)){throw 'Invalid rollback path'};if($entry.HadFile -and (Get-FileHash -LiteralPath (Join-Path $backup $entry.BackupFile)).Hash -ne $entry.SHA256){throw 'Backup integrity failed'}}
foreach($entry in $record.Entries){$target=Join-Path $game $entry.RelativePath;if(Test-Path -LiteralPath $target){Copy-Item -LiteralPath $target -Destination (Join-Path $backup ('before-rollback-'+[guid]::NewGuid().ToString('N')+'.backup'))};if($entry.HadFile){New-Item -ItemType Directory -Path (Split-Path -Parent $target) -Force|Out-Null;Copy-Item -LiteralPath (Join-Path $backup $entry.BackupFile) -Destination $target -Force;if((Get-FileHash -LiteralPath $target).Hash -ne $entry.SHA256){throw 'Restoration verification failed'}}elseif(Test-Path -LiteralPath $target){Remove-Item -LiteralPath $target}}
Write-Output 'Previous files and configuration restored. Backup and pre-rollback files retained.'
