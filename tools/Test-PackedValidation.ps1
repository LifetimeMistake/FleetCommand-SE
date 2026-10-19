param([string]$PackedScript = (Join-Path $PSScriptRoot '../artifacts/validation-script/FleetCommand.Validation/script.cs'))
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$checkDirectory = Join-Path $repoRoot 'artifacts/packed-compile'
New-Item -ItemType Directory -Force -Path $checkDirectory | Out-Null
# Mirror the programmable block wrapper's imports. Source-project using directives
# are not available in the pasted script. In particular, no System.Globalization.
$prefix = @'
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Sandbox.ModAPI.Ingame;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.GUI.TextPanel;
using VRageMath;
public sealed class Program : MyGridProgram {
'@
$source = $prefix + [Environment]::NewLine + [System.IO.File]::ReadAllText($PackedScript) + [Environment]::NewLine + '}'
[System.IO.File]::WriteAllText((Join-Path $checkDirectory 'PackedScript.cs'), $source)
$project = @'
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup><TargetFramework>net48</TargetFramework><LangVersion>6</LangVersion><PlatformTarget>x64</PlatformTarget></PropertyGroup>
  <Import Project="../../build/SpaceEngineers.References.props" />
  <ItemGroup><PackageReference Include="Microsoft.Net.Compilers.Toolset" Version="4.12.0" PrivateAssets="all" /></ItemGroup>
</Project>
'@
$projectPath = Join-Path $checkDirectory 'PackedCompile.csproj'
[System.IO.File]::WriteAllText($projectPath, $project)
& dotnet build $projectPath --nologo -v minimal
if ($LASTEXITCODE -ne 0) { throw 'Packed script does not compile in the programmable block wrapper.' }
