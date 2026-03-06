#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

class IndexerSubsystem : public frc2::SubsystemBase {
 public:
  explicit IndexerSubsystem();

  void FeedToStorage();
  void FeedToShooter();
  void Reverse();
  void Stop();
  void SetPercent(double percent);

 private:
  rev::spark::SparkMax m_motor;
};
