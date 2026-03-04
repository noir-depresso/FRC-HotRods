// PistonSubsystem.h
#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsModuleType.h>

class PistonSubsystem : public frc2::SubsystemBase {
 public:
  PistonSubsystem();

  void Extend();
  void Retract();
  void Off();
  void Toggle();
  bool IsExtended() const;

 private:
  frc::DoubleSolenoid m_solenoid;
  frc::DoubleSolenoid::Value m_state{frc::DoubleSolenoid::Value::kOff};
};