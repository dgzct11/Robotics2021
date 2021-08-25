// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  TalonSRX leftSpeedC = new TalonSRX(Constants.left_motor_port);
  TalonSRX rightSpeedC = new TalonSRX(Constants.right_motor_port);
  
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    
  }

  public void setRightMotor(double speed){
    rightSpeedC.set(ControlMode.PercentOutput, speed);
  }
  public void setLeftMotor(double speed){
    leftSpeedC.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    rightSpeedC.set(ControlMode.PercentOutput, 0);
    leftSpeedC.set(ControlMode.PercentOutput, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
