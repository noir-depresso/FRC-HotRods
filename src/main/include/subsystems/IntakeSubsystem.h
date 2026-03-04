//Intake.h

#pragma once

#include <rev/SparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <optional>


class IntakeSubsystem : public frc2::SubsystemBase {

 public:
  explicit IntakeSubsystem();

  void In();
  void Out();
  void Stop();
  void SetPercent(double percent);
    void EnableAprilTagDirectionControl(bool enable);
  bool IsAprilTagDirectionControlEnabled() const;

  void Periodic() override;

 private:
  rev::spark::SparkMax m_motor;

    double GetClosestAprilTagDistanceMeters() const;
    
      bool m_aprilTagDirectionControlEnabled = false;
  std::optional<double> m_lastTagDistanceMeters;
};