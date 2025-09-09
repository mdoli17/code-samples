#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Perception/AIPerceptionComponent.h"
#include "Perception/AISenseConfig_Sight.h"
#include "VisionModelComponent.generated.h"

class UAISense_Sight;
class UAIPerceptionComponent;
class UCurveFloat;

// Pick which bone-axis is "forward"
UENUM(BlueprintType)
enum class EVisionForwardAxis : uint8
{
	X UMETA(DisplayName="X (ForwardVector)"),
	Y UMETA(DisplayName="Y (RightVector)"),
	Z UMETA(DisplayName="Z (UpVector)")
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnVisionClue, AActor*, Target, FVector, LastKnownPos);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVisionDetected, AActor*, Target);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnVisionLost, AActor*, Target);

UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent), Blueprintable)
class FAITHINABYSS_API UVisionModelComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	UVisionModelComponent();

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

protected:
#pragma region Properties

#pragma region Config - Sources

	UPROPERTY(EditAnywhere, Category="Vision|Source")
	FName HeadSocketName = TEXT("head");

	UPROPERTY(EditAnywhere, Category="Vision|Source")
	EVisionForwardAxis ForwardAxis = EVisionForwardAxis::X;

#pragma endregion

#pragma region Config - FOV

	UPROPERTY(EditAnywhere, Category="Vision|FOV", meta=(ClampMin=0.0, ClampMax=180.0))
	float HorizontalHalfAngleDeg = 55.f;

	UPROPERTY(EditAnywhere, Category="Vision|FOV", meta=(ClampMin=0.0, ClampMax=90.0))
	float VerticalHalfAngleDeg = 40.f;

	UPROPERTY(EditAnywhere, Category="Vision|FOV", meta=(ClampMin=0.0, ClampMax=180.0))
	float AngleInnerDeg = 35.f;

	UPROPERTY(EditAnywhere, Category="Vision|FOV", meta=(ClampMin=0.0, ClampMax=180.0))
	float AngleOuterDeg = 55.f;

#pragma endregion

#pragma region Config - Distance

	// Full weight until here
	UPROPERTY(EditAnywhere, Category="Vision|Distance", meta=(ClampMin=0.0))
	float DistanceStart = 1200.f;

	// 0 weight at/after this
	UPROPERTY(EditAnywhere, Category="Vision|Distance", meta=(ClampMin=0.0))
	float DistanceMax = 3000.f;

#pragma endregion

#pragma region Config - Weights

	UPROPERTY(EditAnywhere, Category="Vision|Weights")
	float AngleExp = 1.0f;

	UPROPERTY(EditAnywhere, Category="Vision|Weights")
	float DistExp = 1.0f;

	UPROPERTY(EditAnywhere, Category="Vision|Weights")
	float VertExp = 1.0f;

	UPROPERTY(EditAnywhere, Category="Vision|Weights")
	float OcclExp = 1.0f;

#pragma endregion

#pragma region Config - Curves

	UPROPERTY(EditAnywhere, Category="Vision|Curves")
	TObjectPtr<UCurveFloat> DistanceCurve = nullptr;

	UPROPERTY(EditAnywhere, Category="Vision|Curves")
	TObjectPtr<UCurveFloat> AngleCurve = nullptr;

	// Maps [Weight (0..1) -> Gain multiplier (0..1+)] lets you deadzone or slow edges]
	UPROPERTY(EditAnywhere, Category="Vision|Curves")
	TObjectPtr<UCurveFloat> WeightToGainMul = nullptr;

	// Maps [Weight (0..1) -> Max allowed score cap (0..1)]
	UPROPERTY(EditAnywhere, Category="Vision|Curves")
	TObjectPtr<UCurveFloat> WeightToScoreCap = nullptr;

#pragma endregion

#pragma region Config - Occlusion

	UPROPERTY(EditAnywhere, Category="Vision|Occlusion", meta=(ClampMin=0))
	int32 OcclusionRays = 3;

	UPROPERTY(EditAnywhere, Category="Vision|Occlusion", meta=(ClampMin=0.0))
	float OcclusionJitterRadius = 12.f;

	UPROPERTY(EditAnywhere, Category="Vision|Occlusion")
	TEnumAsByte<ECollisionChannel> OcclusionTraceChannel = ECC_Visibility;

#pragma endregion

#pragma region Temporal integration

	// How fast the bar fills per second when seen (scaled by w)
	UPROPERTY(EditAnywhere, Category="Vision|Temporal", meta=(ClampMin=0.0))
	float GainRate = 2.0f;

	// How fast the bar drains per second when not seen
	UPROPERTY(EditAnywhere, Category="Vision|Temporal", meta=(ClampMin=0.0))
	float DecayRate = 1.0f;

	// Ignore tiny noise
	UPROPERTY(EditAnywhere, Category="Vision|Temporal")
	float MinWeightDeadzone = 0.02f;

#pragma endregion

#pragma region Config - Smoothing

	// Higher = less smoothing
	UPROPERTY(EditAnywhere, Category="Vision|Smoothing", meta=(ClampMin=0.0))
	float CutoffHz = 6.0f;

#pragma endregion

#pragma region Config - Thresholds

    // Threshold to trigger "OnClue" event
	UPROPERTY(EditAnywhere, Category="Vision|Thresholds", meta=(ClampMin=0.0, ClampMax=1.0))
	float ClueUp = 0.35f;

    // Threshold to trigger "OnClueLost" event
	UPROPERTY(EditAnywhere, Category="Vision|Thresholds", meta=(ClampMin=0.0, ClampMax=1.0))
	float ClueDown = 0.20f;

    // Threshold to trigger "OnDetected" event
	UPROPERTY(EditAnywhere, Category="Vision|Thresholds", meta=(ClampMin=0.0, ClampMax=1.0))
	float DetectUp = 0.70f;

    // Threshold to trigger "OnLost" event
	UPROPERTY(EditAnywhere, Category="Vision|Thresholds", meta=(ClampMin=0.0, ClampMax=1.0))
	float DetectDown = 0.55f;

#pragma endregion

#pragma region Config - Debug

	UPROPERTY(EditAnywhere, Category="Vision|Debug")
	bool bDebugDraw = false;

	UPROPERTY(EditAnywhere, Category="Vision|Debug")
	float DebugTargetRadius;

	UPROPERTY(EditAnywhere, Category="Vision|Debug")
	FColor DebugTargetColor;

	UPROPERTY(EditAnywhere, Category="Vision|Debug")
	float DebugTargetThickness;

#pragma endregion

#pragma endregion

public:
#pragma region Events

    // Target is seen, but not fully detected yet
	UPROPERTY(BlueprintAssignable, Category="Vision|Events")
	FOnVisionClue OnClue;

    // Target which wasn't completely detected yet, is now lost
	UPROPERTY(BlueprintAssignable, Category="Vision|Events")
	FOnVisionLost OnClueLost;	

    // Target is fully detected
	UPROPERTY(BlueprintAssignable, Category="Vision|Events")
	FOnVisionDetected OnDetected;

    // Target which was fully detected, is now lost
	UPROPERTY(BlueprintAssignable, Category="Vision|Events")
	FOnVisionLost OnLost;

#pragma endregion

#pragma region Public API

	/// Initialization function
	UFUNCTION(BlueprintCallable, Category="Vision|Runtime")
	void UpdatePerceptionComponent(UAIPerceptionComponent* NewPerception)
	{
		Perception = NewPerception;

		const FAISenseID AISenseID_Sight = UAISense::GetSenseID<UAISense_Sight>();
		const UAISenseConfig_Sight* SightConfig = Cast<UAISenseConfig_Sight>(Perception->GetSenseConfig(AISenseID_Sight));
		SightRadius = SightConfig->SightRadius;
		LoseSightRadius = SightConfig->LoseSightRadius;
	}

	/// ------- Public getters for UI/BT/StateTree ------- 

	UFUNCTION(BlueprintCallable, Category="Vision|Runtime")
	AActor* GetBestTarget() const { return BestTarget.Get(); }

	UFUNCTION(BlueprintCallable, Category="Vision|Runtime")
	float GetBestVisionScore() const { return BestScoreCached; }

	UFUNCTION(BlueprintCallable, Category="Vision|Runtime")
	float GetVisionScoreFor(AActor* Target) const;

	UFUNCTION(BlueprintCallable, Category="Vision|Runtime")
	FVector GetSmoothedEyeLocation() const { return SmoothedPos; }

	UFUNCTION(BlueprintCallable, Category="Vision|Runtime")
	FQuat GetSmoothedEyeRotation() const { return SmoothedRot; }

#pragma endregion

protected:
	virtual void BeginPlay() override;
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

private:
	TWeakObjectPtr<UAIPerceptionComponent> Perception;

	FVector SmoothedPos = FVector::ZeroVector;
	FQuat SmoothedRot = FQuat::Identity;

	float SightRadius;
	float LoseSightRadius;

#pragma region Weight Scoring

#pragma  region Per Target Fields

	// Score range [0,1]
	TMap<TWeakObjectPtr<AActor>, float> VisionScorePerTarget;
	TMap<TWeakObjectPtr<AActor>, FVector> LastKnownPosPerTarget;

#pragma endregion

	TSet<TWeakObjectPtr<AActor>> ClueSet;
	TSet<TWeakObjectPtr<AActor>> DetectedSet;

	// Cached "best"
	TWeakObjectPtr<AActor> BestTarget;
	float BestScoreCached = 0.f;

#pragma endregion

	/* ---------- Internals ---------- */

	void UpdateSmoothedEyes(float Dt);
	FTransform GetHeadSocketWorld() const;
	FVector GetForwardFromQuat(const FQuat& Q) const;

	void TickTarget(AActor* Target, float Dt);
	float ComputeVisibilityWeight(AActor* Target) const;
	float SampleOcclusion(const FVector& EyePos, const FVector& TgtPos) const;

	void HandleThresholds(AActor* Target, float SNow);
	void UpdateBestTarget();

	/*  ---------- Debug helpers ---------- */

	void DebugDrawFOV() const;
	void DebugDrawTarget(AActor* Target, float Weight, float S) const;
};
