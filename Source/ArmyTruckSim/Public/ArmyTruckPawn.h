#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "ArmyTruckPawn.generated.h"

class UBoxComponent;
class USpringArmComponent;
class UCameraComponent;
class UStaticMeshComponent;
class USpotLightComponent;
class UStaticMesh;

/** Runtime state of one wheel (not reflected). */
struct FTruckWheel
{
	/** Top of the suspension strut, chassis space (cm). */
	FVector LocalMount = FVector::ZeroVector;
	bool bSteered = false;
	bool bDriven = true;
	bool bHandbrake = false;
	int32 Axle = 0;
	int32 Side = 0; // -1 left, +1 right

	double Compression = 0.0; // m
	double PrevCompression = 0.0;
	bool bGrounded = false;
	bool bSlipping = false;
	FVector ContactPoint = FVector::ZeroVector;
	FVector ContactNormal = FVector::UpVector;
	double LoadN = 0.0;
	double AngularVelocity = 0.0; // rad/s
	double SpinDeg = 0.0;
	double VisualCompression = 0.0;

	USceneComponent* SteerPivot = nullptr;
	USceneComponent* SpinPivot = nullptr;
};

/**
 * 5 ton 6x6 army cargo truck (M939 style) with a self contained raycast vehicle model:
 *  - one rigid body (box) with mass = curb + payload and a lowered centre of mass,
 *  - six spring/damper struts with bump stops and anti-roll bars,
 *  - slip angle based lateral tyre force with a friction circle,
 *  - turbo diesel torque curve, torque converter, 5 speed automatic or manual gearbox,
 *    2 speed transfer case (low range for steep hairpins), service brakes, handbrake and engine braking,
 *  - chase and cab cameras, headlights.
 * All visuals are built from engine basic shapes, so the project needs no imported assets.
 */
UCLASS()
class ARMYTRUCKSIM_API AArmyTruckPawn : public APawn
{
	GENERATED_BODY()

public:
	AArmyTruckPawn();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaSeconds) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

	/** Places the truck back on the nearest road, upright and at rest. */
	UFUNCTION(BlueprintCallable, Category = "Truck")
	void ResetToRoad();

	// ---- Telemetry (HUD).
	double GetSpeedKmh() const { return ForwardSpeed * 3.6; }
	double GetEngineRPM() const { return EngineRPM; }
	int32 GetGear() const { return CurrentGear; }
	int32 GetNumGears() const { return GearRatios.Num(); }
	bool IsAutomatic() const { return bAutomatic; }
	bool IsLowRange() const { return bLowRange; }
	bool IsHandbrakeOn() const { return bHandbrake; }
	bool AreLightsOn() const { return bLightsOn; }
	bool IsFlipped() const { return FlippedTime > 1.5f; }
	bool IsAnyWheelSlipping() const;
	double GetThrottle() const { return Throttle; }
	double GetBrake() const { return Brake; }
	double GetSteerAngle() const { return SteerAngle; }
	double GetGradePercent() const;
	double GetTotalMassKg() const { return CurbMassKg + PayloadKg; }
	FVector GetBodyHalfExtent() const;

	// ---- Mass.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Mass")
	double CurbMassKg = 9800.0;

	/** Rated payload of a 5 ton truck. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Mass", meta = (ClampMin = "0", ClampMax = "10000"))
	double PayloadKg = 4536.0;

	/** Offset from the chassis box centre (cm). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Mass")
	FVector CenterOfMassOffset = FVector(-20.0, 0.0, -45.0);

	// ---- Suspension.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Suspension")
	double SuspensionRestLength = 0.45; // m

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Suspension")
	double SuspensionFrequency = 1.4; // Hz

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Suspension")
	double SuspensionDampingRatio = 0.45;

	/** N per metre of compression difference across an axle. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Suspension")
	double AntiRollStiffness = 90000.0;

	// ---- Tyres.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Tyres")
	double WheelRadius = 0.55; // m

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Tyres")
	double TireFriction = 0.9;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Tyres")
	double PeakSlipAngleDeg = 8.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Tyres")
	double RollingResistance = 0.015;

	// ---- Engine (Cummins 6CTA8.3 class turbo diesel).
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Engine")
	double MaxTorqueNm = 1100.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Engine")
	double PeakTorqueRPM = 1500.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Engine")
	double IdleRPM = 650.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Engine")
	double MaxRPM = 2600.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Engine")
	double EngineBrakeTorqueNm = 380.0;

	// ---- Driveline.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	TArray<double> GearRatios;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double ReverseRatio = 6.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double FinalDriveRatio = 6.44;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double LowRangeRatio = 2.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double DrivetrainEfficiency = 0.85;

	/** Torque converter multiplication at stall (automatic only). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double TorqueConverterStallRatio = 1.6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double ShiftTime = 0.45;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double ShiftUpRPM = 2300.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Driveline")
	double ShiftDownRPM = 1100.0;

	// ---- Brakes & steering.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Brakes")
	double MaxBrakeForcePerWheelN = 14000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Brakes")
	double HandbrakeForcePerWheelN = 16000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Steering")
	double MaxSteerAngleDeg = 38.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Steering")
	double SteerSpeedDegPerSec = 70.0;

	/** Fraction of full lock available at 80 km/h. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Steering")
	double HighSpeedSteerFactor = 0.3;

	/** 0.5 * rho * Cd * A (N per (m/s)^2). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Truck|Aero")
	double DragFactor = 4.0;

protected:
	// Input.
	void OnThrottle(float Value) { RawThrottle = FMath::Clamp(Value, 0.f, 1.f); }
	void OnBrake(float Value) { RawBrake = FMath::Clamp(Value, 0.f, 1.f); }
	void OnSteer(float Value) { SteerInput = FMath::Clamp(Value, -1.f, 1.f); }
	void OnLookYaw(float Value);
	void OnLookPitch(float Value);
	void OnHandbrakePressed() { bHandbrake = true; }
	void OnHandbrakeReleased() { bHandbrake = false; }
	void OnShiftUp();
	void OnShiftDown();
	void OnToggleTransmission();
	void OnToggleLowRange();
	void OnToggleCamera();
	void OnToggleLights();
	void OnToggleHelp();
	void OnToggleMap();

	// Simulation steps.
	void UpdateSteering(float Dt);
	void UpdateGearLogic(float Dt);
	void UpdateEngine(float Dt);
	void SimulateWheels(float Dt);
	void UpdateWheelVisuals(float DeltaSeconds);
	void UpdateCameras(float DeltaSeconds);
	void SetGear(int32 NewGear);
	double GetGearRatio(int32 Gear) const;
	double EvaluateTorqueCurve(double RPM) const;

	UStaticMeshComponent* AddPart(const FString& Name, USceneComponent* Parent, UStaticMesh* Mesh, const FVector& LocM, const FVector& SizeM, const FRotator& Rot, const FLinearColor& Color);

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Truck")
	TObjectPtr<UBoxComponent> Chassis;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Truck")
	TObjectPtr<USpringArmComponent> SpringArm;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Truck")
	TObjectPtr<UCameraComponent> ChaseCamera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Truck")
	TObjectPtr<UCameraComponent> CabCamera;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Truck")
	TObjectPtr<USpotLightComponent> HeadlightLeft;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Truck")
	TObjectPtr<USpotLightComponent> HeadlightRight;

private:
	TArray<FTruckWheel> Wheels;
	TArray<TPair<UStaticMeshComponent*, FLinearColor>> PaintJobs;

	// Inputs.
	float RawThrottle = 0.f;
	float RawBrake = 0.f;
	float SteerInput = 0.f;
	float LookYawInput = 0.f;
	float LookPitchInput = 0.f;
	bool bHandbrake = false;

	// Driveline state.
	double Throttle = 0.0;
	double Brake = 0.0;
	double SteerAngle = 0.0;
	double EngineRPM = 650.0;
	double WheelDriveTorque = 0.0; // N*m at the wheels, total
	double ForwardSpeed = 0.0;     // m/s
	int32 CurrentGear = 1;         // -1 = R, 0 = N, 1..N
	bool bAutomatic = true;
	bool bLowRange = false;
	double ShiftTimer = 0.0;
	double DirectionChangeTimer = 0.0;

	// Cameras & misc.
	bool bCabView = false;
	bool bLightsOn = false;
	float CameraYaw = 0.f;
	float CameraPitch = 0.f;
	float LookIdleTime = 10.f;
	float FlippedTime = 0.f;
};
