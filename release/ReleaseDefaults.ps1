function Read-ReleaseDefaults([string]$ProfilePath) {
    $profile=Get-Content -LiteralPath $ProfilePath -Raw -Encoding UTF8 | ConvertFrom-Json
    if($profile.Schema -ne 1 -or !$profile.Id -or !$profile.Version -or
       $profile.Status -notin @('candidate','validated') -or !@($profile.Settings).Count) {
        throw 'Invalid release defaults header.'
    }
    $seen=[Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
    foreach($row in $profile.Settings) {
        if(!$row.Section -or !$row.Key -or !$row.Type -or $null -eq $row.Value -or
           $row.Section.IndexOfAny([char[]]"`r`n[]`0") -ge 0 -or
           $row.Key.IndexOfAny([char[]]"`r`n[]=`0") -ge 0 -or
           $row.Value.IndexOfAny([char[]]"`r`n`0") -ge 0) { throw 'Invalid release default entry.' }
        if(!$seen.Add($row.Section+"`0"+$row.Key)) { throw 'Duplicate release default.' }
    }
    return $profile
}

function Get-EmbeddedDefaultsHash([string]$AssemblyPath) {
    $assembly=[Reflection.Assembly]::Load([IO.File]::ReadAllBytes($AssemblyPath))
    $stream=$assembly.GetManifestResourceStream('BetterPenetrationNext.release-defaults.json')
    if(!$stream) { throw 'The DLL has no embedded release defaults.' }
    $sha=[Security.Cryptography.SHA256]::Create()
    try { return [BitConverter]::ToString($sha.ComputeHash($stream)).Replace('-','') }
    finally { $sha.Dispose(); $stream.Dispose() }
}

function Expand-ReleaseDefault([string]$Value,[string]$GameRoot) {
    if(!$Value.StartsWith('@game/',[StringComparison]::Ordinal)) { return $Value }
    $root=[IO.Path]::GetFullPath($GameRoot).TrimEnd('\','/')
    $path=[IO.Path]::GetFullPath((Join-Path $root $Value.Substring(6).Replace('/',[IO.Path]::DirectorySeparatorChar)))
    if(!$path.StartsWith($root+[IO.Path]::DirectorySeparatorChar,[StringComparison]::OrdinalIgnoreCase)) {
        throw 'A release default path leaves the game directory.'
    }
    return $path
}

function Merge-ReleaseDefaults([AllowEmptyString()][string]$Text,$Profile,[string]$GameRoot) {
    $values=[Collections.Generic.Dictionary[string,string]]::new([StringComparer]::Ordinal)
    foreach($row in $Profile.Settings) {
        $values.Add($row.Section+"`0"+$row.Key,(Expand-ReleaseDefault $row.Value $GameRoot))
    }
    $written=[Collections.Generic.HashSet[string]]::new([StringComparer]::Ordinal)
    $lines=[Collections.Generic.List[string]]::new()
    $section=''
    foreach($line in ($Text -split '\r?\n')) {
        if($line -match '^\s*\[([^\]]+)\]\s*$') { $section=$Matches[1] }
        elseif($line -match '^\s*([^#;=]+?)\s*=') {
            $key=$Matches[1].Trim(); $identity=$section+"`0"+$key
            if($values.ContainsKey($identity)) {
                if($written.Add($identity)) { $lines.Add($key+' = '+$values[$identity]) }
                continue
            }
        }
        $lines.Add($line)
    }
    $section=''
    foreach($row in $Profile.Settings) {
        $identity=$row.Section+"`0"+$row.Key
        if(!$written.Add($identity)) { continue }
        if($section -cne $row.Section) { $lines.Add(''); $lines.Add('['+$row.Section+']'); $section=$row.Section }
        $lines.Add($row.Key+' = '+$values[$identity])
    }
    return ($lines -join [Environment]::NewLine).TrimEnd("`r","`n")+[Environment]::NewLine
}

function Write-ReleaseDefaults([string]$ConfigPath,$Profile,[string]$GameRoot) {
    $path=[IO.Path]::GetFullPath($ConfigPath)
    $text=if(Test-Path -LiteralPath $path) { [IO.File]::ReadAllText($path) } else { '' }
    $merged=Merge-ReleaseDefaults $text $Profile $GameRoot
    $null=New-Item -ItemType Directory -Force (Split-Path -Parent $path)
    $temporary=$path+'.'+[guid]::NewGuid().ToString('N')+'.tmp'
    try {
        [IO.File]::WriteAllText($temporary,$merged,[Text.UTF8Encoding]::new($false))
        if(Test-Path -LiteralPath $path) { [IO.File]::Replace($temporary,$path,[NullString]::Value) }
        else { [IO.File]::Move($temporary,$path) }
    } finally {
        if(Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary }
    }
}
