#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  void In();
  void Out();
  void Stop();
  void SetPercent(double percent);

 private:
  rev::spark::SparkMax m_motor;
};
