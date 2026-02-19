//Intake.h

#pragma once

#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>

class IntakeSubsystem : public frc2::SubsystemBase {

 public:
  explicit IntakeSubsystem();

  void In();
  void Out();
  void Stop();
  void SetPercent(double percent);

 private:
  rev::spark::SparkMax m_motor;
};