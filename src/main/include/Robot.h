#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <photon/PhotonCamera.h>
#include <frc/Servo.h>
#include <frc/controller/PIDController.h>
#include "constants.h"

using namespace CameraConstants;

class Robot : public frc::TimedRobot {
 public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  // 2. ADD THIS LINE HERE
  frc::PIDController m_yawPIDController {kYawP,kYawI,kYawD};
  frc::PIDController m_pitchPIDController {kPitchP,kPitchI,kPitchD};

  photon::PhotonCamera camera{"Camera1"}; 
  frc::Servo yawServo {0};
  frc::Servo pitchServo {1};
  uint8_t yawDirection;
  uint8_t pitchDirection;
  double referenceYaw;
  double referencePitch;
  int counter;



  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};