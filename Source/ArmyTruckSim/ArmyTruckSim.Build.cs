using UnrealBuildTool;

public class ArmyTruckSim : ModuleRules
{
	public ArmyTruckSim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[]
		{
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"PhysicsCore",
			"ProceduralMeshComponent"
		});
	}
}
