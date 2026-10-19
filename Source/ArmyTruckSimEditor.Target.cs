using UnrealBuildTool;
using System.Collections.Generic;

public class ArmyTruckSimEditorTarget : TargetRules
{
	public ArmyTruckSimEditorTarget(TargetInfo Target) : base(Target)
	{
		Type = TargetType.Editor;
		DefaultBuildSettings = BuildSettingsVersion.Latest;
		IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
		ExtraModuleNames.Add("ArmyTruckSim");
	}
}
