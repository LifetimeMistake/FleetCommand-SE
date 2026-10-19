param([string]$Destination = (Join-Path $PSScriptRoot '../blueprints'))
$ErrorActionPreference = 'Stop'
$fixtureName = 'FleetCommand_Validation_Station_v1'
$source = [xml](Get-Content -LiteralPath (Join-Path $PSScriptRoot '../blueprints/FleetCommand_Validation_v1/bp.sbc') -Raw)
$ship = $source.Definitions.ShipBlueprints.ShipBlueprint
$ship.Id.Subtype = $fixtureName
$ship.DisplayName = $fixtureName
$craft = $ship.CubeGrids.CubeGrid
function Set-Value($node, $name, $value) {
    $child = $node.SelectSingleNode($name)
    if ($null -eq $child) { $child = $source.CreateElement($name); [void]$node.AppendChild($child) }
    $child.InnerText = [string]$value
}
function Add-Block($grid, $type, $subtype, $name, $x, $y, $z, $id, $forward='Forward', $up='Up') {
    $node = $source.CreateElement('MyObjectBuilder_CubeBlock')
    $attribute = $source.CreateAttribute('xsi','type','http://www.w3.org/2001/XMLSchema-instance')
    $attribute.Value = "MyObjectBuilder_$type"; [void]$node.Attributes.Append($attribute)
    foreach ($pair in @(@('SubtypeName',$subtype),@('EntityId',$id),@('CustomName',$name),@('Enabled','true'))) { Set-Value $node $pair[0] $pair[1] }
    $min = $source.CreateElement('Min'); $min.SetAttribute('x',[string]$x); $min.SetAttribute('y',[string]$y); $min.SetAttribute('z',[string]$z); [void]$node.AppendChild($min)
    $orientation = $source.CreateElement('BlockOrientation'); $orientation.SetAttribute('Forward',$forward); $orientation.SetAttribute('Up',$up); [void]$node.AppendChild($orientation)
    [void]$grid.SelectSingleNode('CubeBlocks').AppendChild($node)
    return $node
}
Set-Value $craft 'EntityId' '7100000000001'
Set-Value $craft 'DisplayName' 'FC Validation Craft'
# Keep thruster exhaust/contact damage outside this physics measurement fixture.
Set-Value $craft 'DestructibleBlocks' 'false'
$craft.PositionAndOrientation.Position.x = '0'
$craft.PositionAndOrientation.Position.y = '-1'
$craft.PositionAndOrientation.Position.z = '8.08224887'
$nextBlockId = 7100000000100L
foreach ($block in $craft.CubeBlocks.MyObjectBuilder_CubeBlock) { Set-Value $block 'EntityId' $nextBlockId; $nextBlockId++ }
$port = $craft.CubeBlocks.MyObjectBuilder_CubeBlock | Where-Object CustomName -EQ 'FC Connector'
Set-Value $port 'SubtypeName' 'ConnectorMedium'
$port.Min.x = '-1'; $port.Min.y = '1'; $port.Min.z = '-13'
Set-Value $port 'EntityId' '7100000000500'
Set-Value $port 'ConnectedEntityId' '7100000001200'
Set-Value $port 'Connected' 'true'
Set-Value $port 'IsConnecting' 'true'
Set-Value $port 'IsApproaching' 'false'
Set-Value $port 'IsMaster' 'true'
Set-Value $port 'Strength' '0.1'
for ($z=-7; $z -ge -11; $z--) { $null = Add-Block $craft 'CubeBlock' 'SmallBlockArmorBlock' "Dock nose $z" 0 1 $z $nextBlockId; $nextBlockId++ }
$dockLcd = Add-Block $craft 'TextPanel' 'SmallLCDPanel' 'FC Dock Report' -5 1 4 $nextBlockId
Set-Value $dockLcd 'ContentType' 'TEXT_AND_IMAGE'
Set-Value $dockLcd 'FontSize' '0.5'
$station = $source.CreateElement('CubeGrid')
[void]$station.AppendChild($source.CreateElement('CubeBlocks'))
[void]$ship.CubeGrids.AppendChild($station)
foreach ($pair in @(@('EntityId','7100000001001'),@('GridSizeEnum','Large'),@('IsStatic','true'),@('DisplayName','FC Validation Station'),@('DestructibleBlocks','false'),@('CreatePhysics','true'))) { Set-Value $station $pair[0] $pair[1] }
$pose = $source.CreateElement('PositionAndOrientation')
$pose.InnerXml = '<Position x="0" y="0" z="0"/><Forward x="0" y="0" z="-1"/><Up x="0" y="1" z="0"/>'
[void]$station.AppendChild($pose)
$stationId = 7100000001300L
for ($x=-4; $x -le 4; $x++) { for ($y=-4; $y -le 4; $y++) { $null = Add-Block $station 'CubeBlock' 'LargeBlockArmorBlock' "Station armor $x $y" $x $y -2 $stationId; $stationId++ } }
$null = Add-Block $station 'CubeBlock' 'LargeBlockArmorBlock' 'Port support' 0 0 -1 $stationId; $stationId++
$dock = Add-Block $station 'ShipConnector' 'Connector' 'FC Station Connector' 0 0 0 '7100000001200' 'Backward' 'Up'
Set-Value $dock 'ConnectedEntityId' '7100000000500'
Set-Value $dock 'Connected' 'true'
Set-Value $dock 'IsConnecting' 'true'
Set-Value $dock 'IsApproaching' 'false'
Set-Value $dock 'IsMaster' 'false'
Set-Value $dock 'Strength' '0.1'
$battery = Add-Block $station 'BatteryBlock' 'LargeBlockBatteryBlock' 'FC Station Battery' 1 0 -1 $stationId
Set-Value $battery 'CurrentStoredPower' '1.5'
Set-Value $battery 'ChargeMode' '0'
$folder = Join-Path $Destination $fixtureName
New-Item -ItemType Directory -Force -Path $folder | Out-Null
$settings = New-Object System.Xml.XmlWriterSettings
$settings.Indent = $true
$settings.Encoding = New-Object System.Text.UTF8Encoding($false)
$writer = [System.Xml.XmlWriter]::Create((Join-Path $folder 'bp.sbc'),$settings)
try { $source.Save($writer) } finally { $writer.Dispose() }
Write-Output "Generated: $folder/bp.sbc"
