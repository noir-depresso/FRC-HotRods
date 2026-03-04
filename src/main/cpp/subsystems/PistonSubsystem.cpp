// PistonSubsystem.cpp
#include "subsystems/PistonSubsystem.h"

PistonSubsystem::PistonSubsystem()
  : m_solenoid(
      frc::PneumaticsModuleType::REVPH,
      /*forwardChannel=*/0,
      /*reverseChannel=*/1) {}

void PistonSubsystem::Extend() {
  m_state = frc::DoubleSolenoid::Value::kForward;
  m_solenoid.Set(m_state);
}

void PistonSubsystem::Retract() {
  m_state = frc::DoubleSolenoid::Value::kReverse;
  m_solenoid.Set(m_state);
}

void PistonSubsystem::Off() {
  m_state = frc::DoubleSolenoid::Value::kOff;
  m_solenoid.Set(m_state);
}

void PistonSubsystem::Toggle() {
    m_solenoid.Toggle();
}

bool PistonSubsystem::IsExtended() const {
  return m_state == frc::DoubleSolenoid::Value::kForward;
}