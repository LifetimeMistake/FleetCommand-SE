param(
    [string]$GameData = (Join-Path $env:APPDATA 'SpaceEngineers'),
    [string]$PackedScript = (Join-Path $PSScriptRoot '../artifacts/validation-script/FleetCommand.Validation/script.cs')
)
$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath $PackedScript -PathType Leaf)) { throw 'Build FleetCommand.Validation first to produce the MDK script.' }
function Copy-Verified($source, $destination) {
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $destination) | Out-Null
    if (Test-Path -LiteralPath $destination) {
        if ((Get-FileHash -LiteralPath $source).Hash -eq (Get-FileHash -LiteralPath $destination).Hash) {
            Write-Output "Already current: $destination"
            return
        }
        Copy-Item -LiteralPath $destination -Destination ($destination + '.previous') -Force
    }
    Copy-Item -LiteralPath $source -Destination $destination -Force
    if ((Get-FileHash -LiteralPath $source).Hash -ne (Get-FileHash -LiteralPath $destination).Hash) { throw "Copy verification failed: $destination" }
    Write-Output "Deployed: $destination"
}
foreach ($name in @('FleetCommand_Validation_v1', 'FleetCommand_Validation_Dock_v1', 'FleetCommand_Validation_Station_v1')) {
    $source = Join-Path $PSScriptRoot "../blueprints/$name/bp.sbc"
    $destination = Join-Path $GameData "Blueprints/local/$name/bp.sbc"
    Copy-Verified $source $destination
    # The game can prefer the binary cache over updated XML. Only remove this
    # generated blueprint's exact cache file; leave other blueprints untouched.
    $cache = Join-Path (Split-Path -Parent $destination) 'bp.sbcB5'
    if (Test-Path -LiteralPath $cache -PathType Leaf) { Remove-Item -LiteralPath $cache }
}
Copy-Verified $PackedScript (Join-Path $GameData 'IngameScripts/local/FleetCommand.Validation/Script.cs')
