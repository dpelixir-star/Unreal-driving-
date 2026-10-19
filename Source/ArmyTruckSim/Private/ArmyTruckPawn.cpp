#include "ArmyTruckPawn.h"
#include "ArmyTruckSim.h"
#include "ArmyTruckHUD.h"
#include "TruckSimTypes.h"
#include "WorldGenerator.h"

#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/SpringArmComponent.h"
#include "GameFramework/PlayerController.h"
#include "Camera/CameraComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "Engine/CollisionProfile.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	constexpr double M2CM = 100.0;
	constexpr double Gravity = 9.81;

	// Chassis box centre sits 1.6 m above the ground at rest; all part positions are relative to it.
	constexpr double MountZ = -0.75;
	const double AxleX[3] = { 2.35, -1.35, -2.75 };
	constexpr double TrackHalf = 1.0;
}

AArmyTruckPawn::AArmyTruckPawn()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	GearRatios = { 6.93, 4.18, 2.24, 1.40, 1.00 };

	static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeFinder(TEXT("/Engine/BasicShapes/Cube.Cube"));
	static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderFinder(TEXT("/Engine/BasicShapes/Cylinder.Cylinder"));
	UStaticMesh* Cube = CubeFinder.Object;
	UStaticMesh* Cylinder = CylinderFinder.Object;

	// ---- Physics body.
	Chassis = CreateDefaultSubobject<UBoxComponent>(TEXT("Chassis"));
	Chassis->InitBoxExtent(FVector(385.0, 122.0, 100.0));
	Chassis->SetCollisionProfileName(UCollisionProfile::PhysicsActor_ProfileName);
	Chassis->SetCollisionResponseToChannel(ECC_Camera, ECR_Ignore);
	Chassis->SetSimulatePhysics(true);
	Chassis->SetLinearDamping(0.02f);
	Chassis->SetAngularDamping(0.4f);
	Chassis->SetUseCCD(true);
	Chassis->SetNotifyRigidBodyCollision(true);
	Chassis->SetHiddenInGame(true);
	RootComponent = Chassis;

	// ---- Body work (M939 style). Ground is at Z = -1.6 m.
	const FLinearColor Olive(0.13f, 0.15f, 0.07f);
	const FLinearColor DarkOlive(0.08f, 0.09f, 0.045f);
	const FLinearColor CanvasTop(0.19f, 0.18f, 0.10f);
	const FLinearColor Black(0.02f, 0.02f, 0.02f);
	const FLinearColor GlassColor(0.04f, 0.06f, 0.08f);
	const FLinearColor Lamp(0.9f, 0.85f, 0.6f);
	const FLinearColor TailRed(0.6f, 0.02f, 0.02f);
	const FLinearColor Marking(0.7f, 0.7f, 0.65f);
	const FRotator None = FRotator::ZeroRotator;
	const FRotator AlongX(90.0, 0.0, 0.0); // cylinder axis along X

	AddPart(TEXT("FrameRails"), Chassis, Cube, FVector(-0.2, 0.0, -0.68), FVector(7.3, 0.95, 0.25), None, Black);
	AddPart(TEXT("FrontBumper"), Chassis, Cube, FVector(3.78, 0.0, -0.72), FVector(0.22, 2.45, 0.32), None, DarkOlive);
	AddPart(TEXT("Hood"), Chassis, Cube, FVector(2.85, 0.0, -0.12), FVector(1.5, 1.35, 0.85), None, Olive);
	AddPart(TEXT("Grille"), Chassis, Cube, FVector(3.61, 0.0, -0.15), FVector(0.04, 1.15, 0.7), None, Black);
	AddPart(TEXT("FenderL"), Chassis, Cube, FVector(2.75, -0.98, -0.32), FVector(1.7, 0.55, 0.1), None, Olive);
	AddPart(TEXT("FenderR"), Chassis, Cube, FVector(2.75, 0.98, -0.32), FVector(1.7, 0.55, 0.1), None, Olive);
	AddPart(TEXT("HeadlampL"), Chassis, Cylinder, FVector(3.55, -0.98, -0.18), FVector(0.22, 0.22, 0.08), AlongX, Lamp);
	AddPart(TEXT("HeadlampR"), Chassis, Cylinder, FVector(3.55, 0.98, -0.18), FVector(0.22, 0.22, 0.08), AlongX, Lamp);
	AddPart(TEXT("Cab"), Chassis, Cube, FVector(1.35, 0.0, 0.36), FVector(1.6, 2.3, 1.3), None, Olive);
	AddPart(TEXT("CabRoof"), Chassis, Cube, FVector(1.35, 0.0, 1.04), FVector(1.62, 2.32, 0.08), None, CanvasTop);
	AddPart(TEXT("Windshield"), Chassis, Cube, FVector(2.16, 0.0, 0.66), FVector(0.04, 2.0, 0.55), None, GlassColor);
	AddPart(TEXT("SideWindowL"), Chassis, Cube, FVector(1.45, -1.16, 0.62), FVector(0.8, 0.02, 0.45), None, GlassColor);
	AddPart(TEXT("SideWindowR"), Chassis, Cube, FVector(1.45, 1.16, 0.62), FVector(0.8, 0.02, 0.45), None, GlassColor);
	AddPart(TEXT("DoorMarkL"), Chassis, Cube, FVector(1.35, -1.16, 0.12), FVector(0.3, 0.02, 0.3), FRotator(0.0, 0.0, 45.0), Marking);
	AddPart(TEXT("DoorMarkR"), Chassis, Cube, FVector(1.35, 1.16, 0.12), FVector(0.3, 0.02, 0.3), FRotator(0.0, 0.0, 45.0), Marking);
	AddPart(TEXT("MirrorL"), Chassis, Cube, FVector(2.1, -1.33, 0.55), FVector(0.05, 0.12, 0.38), None, Black);
	AddPart(TEXT("MirrorR"), Chassis, Cube, FVector(2.1, 1.33, 0.55), FVector(0.05, 0.12, 0.38), None, Black);
	AddPart(TEXT("StepL"), Chassis, Cube, FVector(1.95, -1.15, -0.65), FVector(0.4, 0.15, 0.06), None, Black);
	AddPart(TEXT("StepR"), Chassis, Cube, FVector(1.95, 1.15, -0.65), FVector(0.4, 0.15, 0.06), None, Black);
	AddPart(TEXT("ExhaustStack"), Chassis, Cylinder, FVector(0.62, 1.05, 0.55), FVector(0.12, 0.12, 1.5), None, Black);
	AddPart(TEXT("SpareTire"), Chassis, Cylinder, FVector(0.42, 0.0, 0.1), FVector(1.05, 1.05, 0.35), AlongX, Black);
	AddPart(TEXT("FuelTank"), Chassis, Cylinder, FVector(0.25, -1.02, -0.72), FVector(0.55, 0.55, 1.1), AlongX, DarkOlive);
	AddPart(TEXT("ToolBox"), Chassis, Cube, FVector(0.25, 1.02, -0.72), FVector(1.0, 0.45, 0.45), None, DarkOlive);
	AddPart(TEXT("BedFloor"), Chassis, Cube, FVector(-1.62, 0.0, -0.45), FVector(4.5, 2.44, 0.14), None, DarkOlive);
	AddPart(TEXT("CanvasCover"), Chassis, Cube, FVector(-1.62, 0.0, 0.47), FVector(4.4, 2.46, 1.7), None, CanvasTop);
	AddPart(TEXT("Tailgate"), Chassis, Cube, FVector(-3.86, 0.0, -0.2), FVector(0.06, 2.44, 0.5), None, DarkOlive);
	AddPart(TEXT("TailLightL"), Chassis, Cube, FVector(-3.9, -1.05, -0.62), FVector(0.04, 0.18, 0.1), None, TailRed);
	AddPart(TEXT("TailLightR"), Chassis, Cube, FVector(-3.9, 1.05, -0.62), FVector(0.04, 0.18, 0.1), None, TailRed);

	// ---- Wheels: front axle steers, all six driven (6x6), rear tandem has the handbrake.
	for (int32 Axle = 0; Axle < 3; ++Axle)
	{
		for (int32 SideIdx = 0; SideIdx < 2; ++SideIdx)
		{
			FTruckWheel W;
			W.Axle = Axle;
			W.Side = SideIdx == 0 ? -1 : 1;
			W.bSteered = Axle == 0;
			W.bDriven = true;
			W.bHandbrake = Axle > 0;
			W.LocalMount = FVector(AxleX[Axle], W.Side * TrackHalf, MountZ) * M2CM;

			const FString Suffix = FString::Printf(TEXT("%d%s"), Axle, SideIdx == 0 ? TEXT("L") : TEXT("R"));
			USceneComponent* Steer = CreateDefaultSubobject<USceneComponent>(*(TEXT("WheelSteer") + Suffix));
			Steer->SetupAttachment(Chassis);
			Steer->SetRelativeLocation(W.LocalMount - FVector(0.0, 0.0, SuspensionRestLength * M2CM));
			USceneComponent* Spin = CreateDefaultSubobject<USceneComponent>(*(TEXT("WheelSpin") + Suffix));
			Spin->SetupAttachment(Steer);
			AddPart(TEXT("Tire") + Suffix, Spin, Cylinder, FVector::ZeroVector, FVector(1.1, 1.1, 0.42), FRotator(0.0, 0.0, 90.0), Black);
			AddPart(TEXT("HubA") + Suffix, Spin, Cube, FVector::ZeroVector, FVector(0.55, 0.44, 0.1), None, Olive);
			AddPart(TEXT("HubB") + Suffix, Spin, Cube, FVector::ZeroVector, FVector(0.1, 0.44, 0.55), None, Olive);
			W.SteerPivot = Steer;
			W.SpinPivot = Spin;
			Wheels.Add(W);
		}
	}

	// ---- Cameras.
	SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
	SpringArm->SetupAttachment(Chassis);
	SpringArm->SetRelativeLocation(FVector(0.0, 0.0, 150.0));
	SpringArm->SetRelativeRotation(FRotator(-12.0, 0.0, 0.0));
	SpringArm->TargetArmLength = 1500.f;
	SpringArm->SocketOffset = FVector(0.0, 0.0, 280.0);
	SpringArm->bInheritPitch = false;
	SpringArm->bInheritRoll = false;
	SpringArm->bInheritYaw = true;
	SpringArm->bEnableCameraLag = true;
	SpringArm->CameraLagSpeed = 6.f;
	SpringArm->bEnableCameraRotationLag = true;
	SpringArm->CameraRotationLagSpeed = 3.5f;
	SpringArm->bDoCollisionTest = true;
	SpringArm->ProbeSize = 15.f;

	ChaseCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ChaseCamera"));
	ChaseCamera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);
	ChaseCamera->SetFieldOfView(75.f);

	CabCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("CabCamera"));
	CabCamera->SetupAttachment(Chassis);
	CabCamera->SetRelativeLocation(FVector(155.0, -55.0, 72.0)); // driver's seat (left hand drive)
	CabCamera->SetFieldOfView(85.f);
	CabCamera->SetAutoActivate(false);

	// ---- Headlights.
	auto MakeHeadlight = [this](const TCHAR* Name, double Y)
	{
		USpotLightComponent* Light = CreateDefaultSubobject<USpotLightComponent>(Name);
		Light->SetupAttachment(Chassis);
		Light->SetRelativeLocation(FVector(362.0, Y, -18.0));
		Light->SetRelativeRotation(FRotator(-4.0, 0.0, 0.0));
		Light->SetIntensityUnits(ELightUnits::Candelas);
		Light->SetIntensity(25000.f);
		Light->SetAttenuationRadius(8000.f);
		Light->SetInnerConeAngle(14.f);
		Light->SetOuterConeAngle(32.f);
		Light->SetVisibility(false);
		return Light;
	};
	HeadlightLeft = MakeHeadlight(TEXT("HeadlightLeft"), -98.0);
	HeadlightRight = MakeHeadlight(TEXT("HeadlightRight"), 98.0);
}

UStaticMeshComponent* AArmyTruckPawn::AddPart(const FString& Name, USceneComponent* Parent, UStaticMesh* Mesh, const FVector& LocM, const FVector& SizeM, const FRotator& Rot, const FLinearColor& Color)
{
	UStaticMeshComponent* Part = CreateDefaultSubobject<UStaticMeshComponent>(FName(*Name));
	Part->SetupAttachment(Parent);
	Part->SetStaticMesh(Mesh);
	Part->SetRelativeLocation(LocM * M2CM);
	Part->SetRelativeRotation(Rot);
	Part->SetRelativeScale3D(SizeM); // engine basic shapes are 1 m in size
	Part->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	Part->SetGenerateOverlapEvents(false);
	Part->SetCanEverAffectNavigation(false);
	PaintJobs.Add(TPair<UStaticMeshComponent*, FLinearColor>(Part, Color));
	return Part;
}

void AArmyTruckPawn::BeginPlay()
{
	Super::BeginPlay();

	for (const TPair<UStaticMeshComponent*, FLinearColor>& Paint : PaintJobs)
	{
		if (Paint.Key)
		{
			Paint.Key->SetMaterial(0, TruckSim::MakeColorMaterial(this, Paint.Value));
		}
	}

	Chassis->SetMassOverrideInKg(NAME_None, static_cast<float>(GetTotalMassKg()), true);
	Chassis->SetCenterOfMass(CenterOfMassOffset);
	EngineRPM = IdleRPM;
	CurrentGear = 1;
}

FVector AArmyTruckPawn::GetBodyHalfExtent() const
{
	return Chassis ? Chassis->GetScaledBoxExtent() : FVector(385.0, 122.0, 100.0);
}

bool AArmyTruckPawn::IsAnyWheelSlipping() const
{
	for (const FTruckWheel& W : Wheels)
	{
		if (W.bSlipping)
		{
			return true;
		}
	}
	return false;
}

double AArmyTruckPawn::GetGradePercent() const
{
	const FVector Fwd = GetActorForwardVector();
	const double Horizontal = FVector(Fwd.X, Fwd.Y, 0.0).Size();
	return Horizontal > KINDA_SMALL_NUMBER ? Fwd.Z / Horizontal * 100.0 : 0.0;
}

// =============================================================================================
// Input
// =============================================================================================

void AArmyTruckPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);
	check(PlayerInputComponent);

	PlayerInputComponent->BindAxis(TEXT("Throttle"), this, &AArmyTruckPawn::OnThrottle);
	PlayerInputComponent->BindAxis(TEXT("Brake"), this, &AArmyTruckPawn::OnBrake);
	PlayerInputComponent->BindAxis(TEXT("Steer"), this, &AArmyTruckPawn::OnSteer);
	PlayerInputComponent->BindAxis(TEXT("LookYaw"), this, &AArmyTruckPawn::OnLookYaw);
	PlayerInputComponent->BindAxis(TEXT("LookPitch"), this, &AArmyTruckPawn::OnLookPitch);

	PlayerInputComponent->BindAction(TEXT("Handbrake"), IE_Pressed, this, &AArmyTruckPawn::OnHandbrakePressed);
	PlayerInputComponent->BindAction(TEXT("Handbrake"), IE_Released, this, &AArmyTruckPawn::OnHandbrakeReleased);
	PlayerInputComponent->BindAction(TEXT("ShiftUp"), IE_Pressed, this, &AArmyTruckPawn::OnShiftUp);
	PlayerInputComponent->BindAction(TEXT("ShiftDown"), IE_Pressed, this, &AArmyTruckPawn::OnShiftDown);
	PlayerInputComponent->BindAction(TEXT("ToggleTransmission"), IE_Pressed, this, &AArmyTruckPawn::OnToggleTransmission);
	PlayerInputComponent->BindAction(TEXT("ToggleLowRange"), IE_Pressed, this, &AArmyTruckPawn::OnToggleLowRange);
	PlayerInputComponent->BindAction(TEXT("ToggleCamera"), IE_Pressed, this, &AArmyTruckPawn::OnToggleCamera);
	PlayerInputComponent->BindAction(TEXT("ResetTruck"), IE_Pressed, this, &AArmyTruckPawn::ResetToRoad);
	PlayerInputComponent->BindAction(TEXT("ToggleLights"), IE_Pressed, this, &AArmyTruckPawn::OnToggleLights);
	PlayerInputComponent->BindAction(TEXT("ToggleHelp"), IE_Pressed, this, &AArmyTruckPawn::OnToggleHelp);
	PlayerInputComponent->BindAction(TEXT("ToggleMap"), IE_Pressed, this, &AArmyTruckPawn::OnToggleMap);
}

void AArmyTruckPawn::OnLookYaw(float Value)
{
	LookYawInput = Value;
	if (FMath::Abs(Value) > 0.01f)
	{
		LookIdleTime = 0.f;
	}
}

void AArmyTruckPawn::OnLookPitch(float Value)
{
	LookPitchInput = Value;
	if (FMath::Abs(Value) > 0.01f)
	{
		LookIdleTime = 0.f;
	}
}

void AArmyTruckPawn::OnShiftUp()
{
	if (bAutomatic)
	{
		// In automatic, E selects Drive from Neutral/Reverse when (nearly) stopped.
		if (CurrentGear <= 0 && FMath::Abs(ForwardSpeed) < 1.0)
		{
			SetGear(CurrentGear + 1);
		}
		return;
	}
	if (CurrentGear < GearRatios.Num())
	{
		SetGear(CurrentGear + 1);
	}
}

void AArmyTruckPawn::OnShiftDown()
{
	if (bAutomatic)
	{
		if (CurrentGear >= 0 && FMath::Abs(ForwardSpeed) < 1.0)
		{
			SetGear(CurrentGear - 1);
		}
		return;
	}
	if (CurrentGear > 0)
	{
		SetGear(CurrentGear - 1);
	}
	else if (CurrentGear == 0 && FMath::Abs(ForwardSpeed) < 1.0)
	{
		SetGear(-1);
	}
}

void AArmyTruckPawn::OnToggleTransmission()
{
	bAutomatic = !bAutomatic;
	if (bAutomatic && CurrentGear == 0)
	{
		SetGear(1);
	}
}

void AArmyTruckPawn::OnToggleLowRange()
{
	// The transfer case can only be shifted at low speed.
	if (FMath::Abs(ForwardSpeed) < 2.5)
	{
		bLowRange = !bLowRange;
		ShiftTimer = ShiftTime * 2.0;
	}
}

void AArmyTruckPawn::OnToggleCamera()
{
	bCabView = !bCabView;
	ChaseCamera->SetActive(!bCabView);
	CabCamera->SetActive(bCabView);
	CameraYaw = 0.f;
	CameraPitch = 0.f;
}

void AArmyTruckPawn::OnToggleLights()
{
	bLightsOn = !bLightsOn;
	HeadlightLeft->SetVisibility(bLightsOn);
	HeadlightRight->SetVisibility(bLightsOn);
}

void AArmyTruckPawn::OnToggleHelp()
{
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		if (AArmyTruckHUD* HUD = Cast<AArmyTruckHUD>(PC->GetHUD()))
		{
			HUD->ToggleHelp();
		}
	}
}

void AArmyTruckPawn::OnToggleMap()
{
	if (APlayerController* PC = Cast<APlayerController>(GetController()))
	{
		if (AArmyTruckHUD* HUD = Cast<AArmyTruckHUD>(PC->GetHUD()))
		{
			HUD->ToggleMap();
		}
	}
}

// =============================================================================================
// Simulation
// =============================================================================================

void AArmyTruckPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	if (DeltaSeconds <= 0.f || !Chassis || !Chassis->IsSimulatingPhysics())
	{
		return;
	}
	// Clamp the step used for force computations so a hitch does not explode the springs.
	const float Dt = FMath::Clamp(DeltaSeconds, 1.f / 240.f, 1.f / 20.f);

	ForwardSpeed = FVector::DotProduct(Chassis->GetPhysicsLinearVelocity(), GetActorForwardVector()) / M2CM;

	UpdateSteering(Dt);
	UpdateGearLogic(Dt);
	UpdateEngine(Dt);
	SimulateWheels(Dt);
	UpdateWheelVisuals(DeltaSeconds);
	UpdateCameras(DeltaSeconds);

	// Detect a roll over so the HUD can offer a recovery.
	const bool bUpsideDown = GetActorUpVector().Z < 0.3;
	FlippedTime = (bUpsideDown && FMath::Abs(ForwardSpeed) < 2.0) ? FlippedTime + DeltaSeconds : 0.f;
}

void AArmyTruckPawn::UpdateSteering(float Dt)
{
	const double SpeedFactor = FMath::Clamp(FMath::Abs(ForwardSpeed) * 3.6 / 80.0, 0.0, 1.0);
	const double Lock = MaxSteerAngleDeg * FMath::Lerp(1.0, HighSpeedSteerFactor, SpeedFactor);
	const double Target = SteerInput * Lock;
	// Return to centre a little faster than turning in.
	const double Rate = (FMath::Abs(Target) < FMath::Abs(SteerAngle) ? 1.4 : 1.0) * SteerSpeedDegPerSec;
	SteerAngle = FMath::FInterpConstantTo(SteerAngle, Target, Dt, Rate);
}

double AArmyTruckPawn::GetGearRatio(int32 Gear) const
{
	if (Gear < 0)
	{
		return -ReverseRatio;
	}
	if (Gear == 0 || !GearRatios.IsValidIndex(Gear - 1))
	{
		return 0.0;
	}
	return GearRatios[Gear - 1];
}

void AArmyTruckPawn::SetGear(int32 NewGear)
{
	NewGear = FMath::Clamp(NewGear, -1, GearRatios.Num());
	if (NewGear != CurrentGear)
	{
		CurrentGear = NewGear;
		ShiftTimer = ShiftTime;
	}
}

void AArmyTruckPawn::UpdateGearLogic(float Dt)
{
	ShiftTimer = FMath::Max(0.0, ShiftTimer - Dt);

	if (!bAutomatic)
	{
		Throttle = RawThrottle;
		Brake = RawBrake;
		return;
	}

	// Automatic: W = accelerate, S = brake; holding S at a standstill selects reverse (and vice versa).
	if (CurrentGear == 0)
	{
		if (RawThrottle > 0.1f)
		{
			SetGear(1);
		}
		else if (RawBrake > 0.1f && FMath::Abs(ForwardSpeed) < 1.0)
		{
			SetGear(-1);
		}
	}
	else if (CurrentGear > 0)
	{
		const bool bWantsReverse = RawBrake > 0.1f && RawThrottle < 0.05f && ForwardSpeed < 0.5;
		DirectionChangeTimer = bWantsReverse ? DirectionChangeTimer + Dt : 0.0;
		if (DirectionChangeTimer > 0.4)
		{
			SetGear(-1);
			DirectionChangeTimer = 0.0;
		}
		else if (ShiftTimer <= 0.0)
		{
			if (EngineRPM > ShiftUpRPM && CurrentGear < GearRatios.Num())
			{
				SetGear(CurrentGear + 1);
			}
			else if (CurrentGear > 1)
			{
				// Kick down earlier under heavy throttle (climbing hairpins).
				const double DownRPM = FMath::Lerp(ShiftDownRPM, ShiftDownRPM + 350.0, Throttle);
				const double RPMAfter = EngineRPM * GetGearRatio(CurrentGear - 1) / GetGearRatio(CurrentGear);
				if (EngineRPM < DownRPM && RPMAfter < ShiftUpRPM - 150.0)
				{
					SetGear(CurrentGear - 1);
				}
			}
		}
	}
	else
	{
		const bool bWantsForward = RawThrottle > 0.1f && RawBrake < 0.05f && ForwardSpeed > -0.5;
		DirectionChangeTimer = bWantsForward ? DirectionChangeTimer + Dt : 0.0;
		if (DirectionChangeTimer > 0.4)
		{
			SetGear(1);
			DirectionChangeTimer = 0.0;
		}
	}

	if (CurrentGear >= 0)
	{
		Throttle = RawThrottle;
		Brake = RawBrake;
	}
	else
	{
		Throttle = RawBrake;
		Brake = RawThrottle;
	}
}

double AArmyTruckPawn::EvaluateTorqueCurve(double RPM) const
{
	double Normalized;
	if (RPM <= PeakTorqueRPM)
	{
		const double T = FMath::Clamp((RPM - IdleRPM) / FMath::Max(PeakTorqueRPM - IdleRPM, 1.0), 0.0, 1.0);
		Normalized = 0.6 + 0.4 * T;
	}
	else
	{
		const double T = FMath::Clamp((RPM - PeakTorqueRPM) / FMath::Max(MaxRPM - PeakTorqueRPM, 1.0), 0.0, 1.0);
		Normalized = 1.0 - 0.35 * T * T;
	}
	return MaxTorqueNm * Normalized;
}

void AArmyTruckPawn::UpdateEngine(float Dt)
{
	const double Ratio = GetGearRatio(CurrentGear) * FinalDriveRatio * (bLowRange ? LowRangeRatio : 1.0);
	const bool bEngaged = CurrentGear != 0 && ShiftTimer <= 0.0;
	const double WheelOmega = ForwardSpeed / WheelRadius;
	const double CoupledRPM = FMath::Abs(WheelOmega * Ratio) * 60.0 / (2.0 * PI);
	const double StallRPM = IdleRPM + 650.0;

	double TargetRPM;
	if (bEngaged)
	{
		// Torque converter / clutch slip lets the engine rev above wheel speed when launching.
		TargetRPM = FMath::Max(CoupledRPM, IdleRPM + Throttle * (StallRPM - IdleRPM));
	}
	else
	{
		TargetRPM = IdleRPM + Throttle * (MaxRPM - IdleRPM);
	}
	EngineRPM = FMath::FInterpTo(EngineRPM, TargetRPM, Dt, bEngaged ? 12.0 : 4.0);
	EngineRPM = FMath::Clamp(EngineRPM, IdleRPM * 0.9, MaxRPM + 250.0);

	double Torque = Throttle * EvaluateTorqueCurve(EngineRPM);
	if (EngineRPM >= MaxRPM)
	{
		Torque = 0.0; // governor
	}
	if (bEngaged && Throttle < 0.05)
	{
		const double Brake01 = FMath::Clamp((EngineRPM - IdleRPM) / (MaxRPM - IdleRPM), 0.0, 1.0);
		Torque -= EngineBrakeTorqueNm * Brake01;
	}
	double Multiplication = 1.0;
	if (bAutomatic && CoupledRPM < StallRPM)
	{
		Multiplication = FMath::Lerp(TorqueConverterStallRatio, 1.0, CoupledRPM / StallRPM);
	}
	WheelDriveTorque = bEngaged ? Torque * Ratio * DrivetrainEfficiency * Multiplication : 0.0;
}

void AArmyTruckPawn::SimulateWheels(float Dt)
{
	UWorld* World = GetWorld();
	const FTransform TM = Chassis->GetComponentTransform();
	const FVector Up = TM.GetUnitAxis(EAxis::Z);
	const FVector Fwd = TM.GetUnitAxis(EAxis::X);

	const double Mass = Chassis->GetMass();
	const int32 NumWheels = Wheels.Num();
	const double MassPerWheel = Mass / FMath::Max(NumWheels, 1);
	const double Omega = 2.0 * PI * SuspensionFrequency;
	const double K = MassPerWheel * Omega * Omega;
	const double C = 2.0 * SuspensionDampingRatio * FMath::Sqrt(K * MassPerWheel);
	const double MaxLen = SuspensionRestLength + WheelRadius;
	const double PeakSlip = FMath::DegreesToRadians(PeakSlipAngleDeg);

	FCollisionQueryParams Params(SCENE_QUERY_STAT(TruckWheel), false, this);
	const FCollisionObjectQueryParams ObjectParams(ECC_WorldStatic);

	// Pass 1: suspension ray casts.
	int32 DrivenGrounded = 0;
	for (FTruckWheel& W : Wheels)
	{
		const FVector Start = TM.TransformPosition(W.LocalMount);
		const FVector End = Start - Up * (MaxLen * M2CM);
		FHitResult Hit;
		W.PrevCompression = W.Compression;
		W.bSlipping = false;
		if (World->LineTraceSingleByObjectType(Hit, Start, End, ObjectParams, Params))
		{
			W.bGrounded = true;
			W.Compression = FMath::Clamp(MaxLen - Hit.Distance / M2CM, 0.0, MaxLen);
			W.ContactPoint = Hit.ImpactPoint;
			W.ContactNormal = Hit.ImpactNormal;
			DrivenGrounded += W.bDriven ? 1 : 0;
		}
		else
		{
			W.bGrounded = false;
			W.Compression = 0.0;
			W.LoadN = 0.0;
		}
	}

	const double DriveForcePerWheel = DrivenGrounded > 0 ? (WheelDriveTorque / WheelRadius) / DrivenGrounded : 0.0;

	// Pass 2: suspension and tyre forces.
	for (FTruckWheel& W : Wheels)
	{
		if (!W.bGrounded)
		{
			W.AngularVelocity *= 0.98;
			continue;
		}

		const double CompressionVelocity = (W.Compression - W.PrevCompression) / Dt;
		double Fs = K * W.Compression + C * CompressionVelocity;
		const double BumpStart = SuspensionRestLength * 0.85;
		if (W.Compression > BumpStart)
		{
			Fs += K * 6.0 * (W.Compression - BumpStart);
		}
		Fs = FMath::Max(Fs, 0.0);
		W.LoadN = Fs;

		FVector WheelFwd = Fwd;
		if (W.bSteered)
		{
			WheelFwd = Fwd.RotateAngleAxis(SteerAngle, Up);
		}
		const FVector N = W.ContactNormal;
		const FVector LongDir = (WheelFwd - N * FVector::DotProduct(WheelFwd, N)).GetSafeNormal();
		const FVector LatDir = FVector::CrossProduct(N, LongDir);
		const FVector ContactVel = Chassis->GetPhysicsLinearVelocityAtPoint(W.ContactPoint) / M2CM;
		const double VLong = FVector::DotProduct(ContactVel, LongDir);
		const double VLat = FVector::DotProduct(ContactVel, LatDir);

		// Longitudinal: drive, brakes, rolling resistance.
		double Fx = W.bDriven ? DriveForcePerWheel : 0.0;
		const bool bHandbrakeHere = bHandbrake && W.bHandbrake;
		const double BrakeForce = Brake * MaxBrakeForcePerWheelN + (bHandbrakeHere ? HandbrakeForcePerWheelN : 0.0);
		const double Resist = BrakeForce + RollingResistance * Fs;
		const double StopCap = MassPerWheel * FMath::Abs(VLong) / Dt; // never reverse the wheel's motion
		if (FMath::Abs(VLong) > 0.005)
		{
			Fx -= FMath::Sign(VLong) * FMath::Min(Resist, StopCap);
		}
		if (BrakeForce > 0.0 && FMath::Abs(VLong) < 0.5)
		{
			// Static hold on slopes: cancel gravity along the tyre, up to the brake capacity.
			const double GravityAlong = -Gravity * LongDir.Z;
			Fx += FMath::Clamp(-GravityAlong * MassPerWheel - DriveForcePerWheel * (W.bDriven ? 1.0 : 0.0), -BrakeForce, BrakeForce);
		}

		// Lateral: slip angle model, capped so it never overshoots within one step.
		const double SlipAngle = FMath::Atan2(VLat, FMath::Max(FMath::Abs(VLong), 1.5));
		const double MaxGrip = TireFriction * Fs;
		double Fy = -MaxGrip * FMath::Clamp(SlipAngle / PeakSlip, -1.0, 1.0);
		const double LatCap = MassPerWheel * FMath::Abs(VLat) / Dt * 0.9;
		Fy = FMath::Clamp(Fy, -LatCap, LatCap);

		// Friction circle.
		const double Total = FMath::Sqrt(Fx * Fx + Fy * Fy);
		if (Total > MaxGrip && Total > KINDA_SMALL_NUMBER)
		{
			const double Scale = MaxGrip / Total;
			Fx *= Scale;
			Fy *= Scale;
			W.bSlipping = true;
		}

		const FVector Force = (LongDir * Fx + LatDir * Fy + Up * Fs) * M2CM;
		Chassis->AddForceAtLocation(Force, W.ContactPoint);

		W.AngularVelocity = bHandbrakeHere && FMath::Abs(VLong) < 3.0 ? 0.0 : VLong / WheelRadius;
	}

	// Anti-roll bars.
	for (int32 Axle = 0; Axle < 3; ++Axle)
	{
		FTruckWheel* Left = nullptr;
		FTruckWheel* Right = nullptr;
		for (FTruckWheel& W : Wheels)
		{
			if (W.Axle == Axle)
			{
				(W.Side < 0 ? Left : Right) = &W;
			}
		}
		if (!Left || !Right)
		{
			continue;
		}
		const double F = AntiRollStiffness * (Left->Compression - Right->Compression);
		if (Left->bGrounded)
		{
			Chassis->AddForceAtLocation(Up * (F * M2CM), TM.TransformPosition(Left->LocalMount));
		}
		if (Right->bGrounded)
		{
			Chassis->AddForceAtLocation(Up * (-F * M2CM), TM.TransformPosition(Right->LocalMount));
		}
	}

	// Aerodynamic drag.
	const FVector Vel = Chassis->GetPhysicsLinearVelocity() / M2CM;
	Chassis->AddForce(-Vel * Vel.Size() * DragFactor * M2CM);
}

void AArmyTruckPawn::UpdateWheelVisuals(float DeltaSeconds)
{
	for (FTruckWheel& W : Wheels)
	{
		W.VisualCompression = FMath::FInterpTo(W.VisualCompression, W.Compression, DeltaSeconds, 25.0);
		if (W.SteerPivot)
		{
			const double Drop = (SuspensionRestLength - W.VisualCompression) * M2CM;
			W.SteerPivot->SetRelativeLocation(W.LocalMount - FVector(0.0, 0.0, Drop));
			W.SteerPivot->SetRelativeRotation(FRotator(0.0, W.bSteered ? SteerAngle : 0.0, 0.0));
		}
		W.SpinDeg = FMath::Fmod(W.SpinDeg - FMath::RadiansToDegrees(W.AngularVelocity * DeltaSeconds), 360.0);
		if (W.SpinPivot)
		{
			W.SpinPivot->SetRelativeRotation(FRotator(W.SpinDeg, 0.0, 0.0));
		}
	}
}

void AArmyTruckPawn::UpdateCameras(float DeltaSeconds)
{
	LookIdleTime += DeltaSeconds;
	CameraYaw = FMath::Clamp(CameraYaw + LookYawInput * 2.5f, -170.f, 170.f);
	CameraPitch = FMath::Clamp(CameraPitch + LookPitchInput * 2.0f, -35.f, 30.f);
	if (LookIdleTime > 2.0f)
	{
		CameraYaw = FMath::FInterpTo(CameraYaw, 0.f, DeltaSeconds, 2.f);
		CameraPitch = FMath::FInterpTo(CameraPitch, 0.f, DeltaSeconds, 2.f);
	}
	if (bCabView)
	{
		CabCamera->SetRelativeRotation(FRotator(CameraPitch - 3.f, CameraYaw, 0.f));
	}
	else
	{
		SpringArm->SetRelativeRotation(FRotator(-12.f + CameraPitch, CameraYaw, 0.f));
		// Pull the camera back a little at speed.
		const float TargetLength = 1500.f + FMath::Clamp(static_cast<float>(FMath::Abs(ForwardSpeed)), 0.f, 25.f) * 12.f;
		SpringArm->TargetArmLength = FMath::FInterpTo(SpringArm->TargetArmLength, TargetLength, DeltaSeconds, 1.5f);
	}
}

void AArmyTruckPawn::ResetToRoad()
{
	AWorldGenerator* Gen = AWorldGenerator::Find(this);
	if (!Gen)
	{
		return;
	}
	int32 RoadIdx;
	double Dist, DistSq;
	if (!Gen->FindNearestRoadPoint(GetActorLocation(), RoadIdx, Dist, DistSq))
	{
		return;
	}
	const FSimRoad& Road = Gen->GetRoads()[RoadIdx];
	const FVector Loc = Road.Center.GetLocationAtDistance(Dist);
	FVector Dir = Road.Center.GetDirectionAtDistance(Dist, 400.0);
	if (FVector::DotProduct(Dir, GetActorForwardVector()) < 0.0)
	{
		Dir = -Dir;
	}
	// Drop onto the correct lane side.
	const FVector Flat = Dir.GetSafeNormal2D();
	const FVector Right(-Flat.Y, Flat.X, 0.0);
	const double Side = Gen->bLeftHandTraffic ? -1.0 : 1.0;
	const FVector Target = Loc + Right * (Road.HalfWidth * 0.5 * Side) + FVector(0.0, 0.0, 200.0);
	FRotator Rot = Dir.Rotation();
	Rot.Roll = 0.0;

	Chassis->SetPhysicsLinearVelocity(FVector::ZeroVector);
	Chassis->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
	SetActorLocationAndRotation(Target, Rot, false, nullptr, ETeleportType::TeleportPhysics);
	EngineRPM = IdleRPM;
	SteerAngle = 0.0;
	FlippedTime = 0.f;
	if (bAutomatic)
	{
		SetGear(1);
	}
	for (FTruckWheel& W : Wheels)
	{
		W.Compression = W.PrevCompression = 0.0;
		W.AngularVelocity = 0.0;
	}
}
