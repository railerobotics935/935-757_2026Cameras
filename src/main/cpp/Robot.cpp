
#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/print.h>
#include <iostream>

Robot::Robot() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  yawDirection = 1;
  pitchDirection = 1;
  referenceYaw = 90;
  referencePitch = 90;

//  auto nt_inst = nt::NetworkTableInstance::GetDefault();
//  auto nt_table = nt_inst.GetTable("SmartDashboard");
//  nte_camera_location = nt_table->GetEntry("OakDLite/Object[0]/Location");
//  camera_location_sub = nt_table->GetDoubleTopic("OakDLite/Object[0]/Location").Subscribe(locationObj[0, 1]);
}


void Robot::RobotPeriodic() {
  //std::cout << "Test: " << locationObj[0] << ", " << locationObj[1] << std::endl;
#if 0
  bool targetVisible = false;
  double targetYaw = 0.0;
  double targetPitch = 0.0;
  auto results = camera.GetAllUnreadResults();
  if (results.size() > 0) {
    // Camera processed a new frame since last
    // Get the last one in the list.
    auto result = results[results.size() - 1];
    if (result.HasTargets()) {
      // At least one AprilTag was seen by the camera
      for (auto& target : result.GetTargets()) {
        if (target.GetFiducialId() == 10) {
          // Found Tag 10, record its information
          targetYaw = target.GetYaw();
          targetPitch = target.GetPitch();
          targetVisible = true;
          counter = 0;

          std::cout << "target Yaw: " << targetYaw << std::endl;
          std::cout << "target Pitch: " << targetPitch << std::endl;
          double pidYawOutput = m_yawPIDController.Calculate(-targetYaw, 0.0);
          double pidPitchOutput = m_pitchPIDController.Calculate(targetPitch, 0.0);
          
          std::cout << "PID output: " << pidYawOutput << std::endl;
          std::cout << "Reference input: " << referenceYaw << std::endl;

          if ((pidYawOutput + referenceYaw) > 135) {
            referenceYaw = 135;
          }
          else if ((pidYawOutput + referenceYaw) < 45)
            referenceYaw = 45;
          else
            referenceYaw = referenceYaw + pidYawOutput;

          
          if ((pidPitchOutput + referencePitch) > 120) {
            referencePitch = 120;
          }
          else if ((pidPitchOutput + referencePitch) < 80)
            referencePitch = 80;
          else
            referencePitch = referencePitch + pidPitchOutput;
          

          std::cout << "Reference output: " << referenceYaw << std::endl;
//          referenceYaw = referenceYaw + (targetYaw / 2);
        }
      }
    }
    else if (counter > yawCounterMax)
    {
      
      // search target
      if (yawDirection)
      {
        if (referenceYaw < 135)
        {
          referenceYaw+=2;
        }
        else
        {
          yawDirection = 0;
        }
      }
      else
      {
        if (referenceYaw > 45)
        {
          referenceYaw-=2;
        }
        else
        {
          yawDirection = 1;
        }
      }

      if(counter >= counterMax) {
        counter = counterMax; 
        referencePitch = 90;
      }
     std::cout << referenceYaw << std::endl;
    } else {
      counter++;
    }
  }
#endif

  // Step 1: Get List of Fuel
  //std::cout << "Step 1" << std::endl;
  m_listOfFuel.clear();
  for (int i = 0; i < MAX_NUM_OBJECTS; i++) {
    if (m_OakDLiteCameraSensor.ObjectIsFuel(i) && m_OakDLiteCameraSensor.ObjectIsTracked(i))
    {
      m_listOfFuel.push_back(i);
      //std::cout << "Object added" << std::endl;
    }
  }

  // Step 2: (TODO) eleminate any Fuel too close to robots

  // Step 3: Determine Closest Fuel and update class value
  if (m_listOfFuel.size() > 0) {
    double minDistanceFromRobot = 100000000.0;
    for (int i = 0; i < (int)m_listOfFuel.size(); i++) {
      if (m_OakDLiteCameraSensor.GetDistanceFromRobot(i) < minDistanceFromRobot) {
        minDistanceFromRobot = m_OakDLiteCameraSensor.GetDistanceFromRobot(i);
        //m_translationArr = nte_location[i].GetDoubleArray(std::vector<double>());

        m_bestFuelId = i;
      }
    }
    // return the good note
    std::cout << "FuelID: " << m_bestFuelId << " ,Distance:" << minDistanceFromRobot << std::endl;
//    return m_bestFuelId;
  }
  // If no Notes, return -1
//  else
//    return -1;
}



void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();


  wpi::print("Auto selected: {}\n", m_autoSelected);

  if (m_autoSelected == kAutoNameCustom) {
   
  } else {
 
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    
  } else {
   
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  
  auto result = camera.GetLatestResult();

  if (result.HasTargets()) {
    auto target = result.GetBestTarget();
    
    double yaw = target.GetYaw();
    double pitch = target.GetPitch();

//    frc::SmartDashboard::PutNumber("Vision/Yaw", yaw);
//    frc::SmartDashboard::PutNumber("Vision/Pitch", pitch);
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
  referenceYaw = 90;
}

void Robot::TestPeriodic() {
        yawServo.SetAngle(referenceYaw);
        pitchServo.SetAngle(referencePitch);

}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
