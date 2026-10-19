param([string]$Destination = (Join-Path $PSScriptRoot '../blueprints'))
$ErrorActionPreference = 'Stop'
# Base-game definitions: CubeBlocks_{Armor,Automation,Communications,Control,
# Energy,LCDPanels,Logistics,Thrusters}.sbc. No DLC blocks.
function New-Block($type, $subtype, $name, $x, $y, $z, $forward='Forward', $up='Up', $state='') {
    return "<MyObjectBuilder_CubeBlock xsi:type=`"MyObjectBuilder_$type`"><SubtypeName>$subtype</SubtypeName><Min x=`"$x`" y=`"$y`" z=`"$z`"/><BlockOrientation Forward=`"$forward`" Up=`"$up`"/><CustomName>$name</CustomName><Enabled>true</Enabled>$state</MyObjectBuilder_CubeBlock>"
}
function Save-Blueprint($name, $blocks) {
    $folder = Join-Path $Destination $name
    New-Item -ItemType Directory -Force -Path $folder | Out-Null
    $xml = "<?xml version=`"1.0`"?><Definitions xmlns:xsi=`"http://www.w3.org/2001/XMLSchema-instance`" xmlns:xsd=`"http://www.w3.org/2001/XMLSchema`"><ShipBlueprints><ShipBlueprint xsi:type=`"MyObjectBuilder_ShipBlueprintDefinition`"><Id Type=`"MyObjectBuilder_ShipBlueprintDefinition`" Subtype=`"$name`"/><DisplayName>$name</DisplayName><CubeGrids><CubeGrid><SubtypeName/><EntityId>0</EntityId><PositionAndOrientation><Position x=`"0`" y=`"0`" z=`"0`"/><Forward x=`"0`" y=`"0`" z=`"-1`"/><Up x=`"0`" y=`"1`" z=`"0`"/></PositionAndOrientation><GridSizeEnum>Small</GridSizeEnum><CubeBlocks>$($blocks -join '')</CubeBlocks><IsStatic>false</IsStatic><DisplayName>$name</DisplayName><DestructibleBlocks>true</DestructibleBlocks><CreatePhysics>true</CreatePhysics></CubeGrid></CubeGrids></ShipBlueprint></ShipBlueprints></Definitions>"
    $doc = New-Object System.Xml.XmlDocument
    $doc.LoadXml($xml)
    $settings = New-Object System.Xml.XmlWriterSettings
    $settings.Indent = $true
    $settings.Encoding = New-Object System.Text.UTF8Encoding($false)
    $writer = [System.Xml.XmlWriter]::Create((Join-Path $folder 'bp.sbc'), $settings)
    try { $doc.Save($writer) } finally { $writer.Dispose() }
}
$blocks = @()
for ($x=-5; $x -le 5; $x++) {
    for ($z=-7; $z -le 7; $z++) { $blocks += New-Block 'CubeBlock' 'SmallBlockArmorBlock' "Armor $x $z" $x 0 $z }
}
$blocks += New-Block 'RemoteControl' 'SmallBlockRemoteControl' 'FC Reference' 0 1 -4 'Forward' 'Up' '<DampenersEnabled>false</DampenersEnabled>'
$blocks += New-Block 'Cockpit' 'SmallBlockCockpit' 'FC Cockpit' -1 1 -1 'Forward' 'Up' '<DampenersEnabled>false</DampenersEnabled>'
$blocks += New-Block 'MyProgrammableBlock' 'SmallProgrammableBlock' 'FC Validation' -3 1 2 'Down' 'Forward'
$blocks += New-Block 'BatteryBlock' 'SmallBlockSmallBatteryBlock' 'FC Battery' -3 1 0 'Forward' 'Up' '<CurrentStoredPower>0.025</CurrentStoredPower><ChargeMode>0</ChargeMode>'
$fuel = '<Inventory><Items><MyObjectBuilder_InventoryItem><Amount>10</Amount><PhysicalContent xsi:type="MyObjectBuilder_Ingot"><SubtypeName>Uranium</SubtypeName></PhysicalContent><ItemId>0</ItemId></MyObjectBuilder_InventoryItem></Items><nextItemId>1</nextItemId></Inventory>'
$blocks += New-Block 'Reactor' 'SmallBlockSmallGenerator' 'FC Reactor' -3 1 -3 'Forward' 'Up' $fuel
$blocks += New-Block 'Reactor' 'SmallBlockSmallGenerator' 'FC Reactor B' -4 1 -3 'Forward' 'Up' $fuel
$blocks += New-Block 'Reactor' 'SmallBlockSmallGenerator' 'FC Reactor C' -5 1 -3 'Forward' 'Up' $fuel
$blocks += New-Block 'Gyro' 'SmallBlockGyro' 'FC Gyro A' 3 1 0
$blocks += New-Block 'CubeBlock' 'SmallBlockArmorBlock' 'Gyro support' 2 1 2
$blocks += New-Block 'Gyro' 'SmallBlockGyro' 'FC Gyro B' 3 1 2 'Forward' 'Right'
$blocks += New-Block 'TextPanel' 'SmallLCDPanel' 'FC Report' 0 1 4 'Forward' 'Up' '<ContentType>TEXT_AND_IMAGE</ContentType><FontSize>0.5</FontSize>'
$blocks += New-Block 'ShipConnector' 'ConnectorSmall' 'FC Connector' 0 1 -7
$blocks += New-Block 'Thrust' 'SmallBlockSmallThrust' 'FC Thrust -X' 6 0 0 'Right' 'Up'
$blocks += New-Block 'Thrust' 'SmallBlockSmallThrust' 'FC Thrust +X' -7 0 0 'Left' 'Up'
$blocks += New-Block 'Thrust' 'SmallBlockSmallThrust' 'FC Thrust -Z' 0 0 8 'Backward' 'Up'
$blocks += New-Block 'Thrust' 'SmallBlockSmallThrust' 'FC Thrust +Z' 0 0 -9 'Forward' 'Up'
$blocks += New-Block 'Thrust' 'SmallBlockSmallThrust' 'FC Thrust -Y' 4 1 5 'Up' 'Forward'
$blocks += New-Block 'Thrust' 'SmallBlockSmallThrust' 'FC Thrust +Y' 4 -2 -5 'Down' 'Forward'
Save-Blueprint 'FleetCommand_Validation_v1' $blocks
$dock = @()
$dock += New-Block 'CubeBlock' 'SmallBlockArmorBlock' 'Dock armor' 0 0 0
$dock += New-Block 'ShipConnector' 'ConnectorSmall' 'FC Dock' 0 0 1 'Backward' 'Up'
$dock += New-Block 'BatteryBlock' 'SmallBlockSmallBatteryBlock' 'Dock battery' 1 0 0 'Forward' 'Up' '<CurrentStoredPower>0.025</CurrentStoredPower>'
Save-Blueprint 'FleetCommand_Validation_Dock_v1' $dock
