// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;

public class DriveStraightDistance extends CommandBase {
  /** Creates a new DriveStraightDistance. */
  double distance;
  double angle;
  DriveTrain driveTrain;
  double kp = 0.05;
  double ki = 0;
  double kd = 0;
  double kf = 0;
  double time = 0;
  double previous_error = 0;
  double error = 0;
  double value = 0;
  double startTime = 0;
  public DriveStraightDistance(double d, double a, DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    distance = d;
    driveTrain = dt;
    angle = a;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NavXGyro.ahrs.resetDisplacement();
    RobotContainer.inAuto = true;
    startTime = System.currentTimeMillis()/1000;
    driveTrain.leftSpeedC.setSelectedSensorPosition(0);
      driveTrain.rightSpeedC.setSelectedSensorPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      time = System.currentTimeMillis()/1000-startTime+0.04;
      
      driveTrain.leftSpeedC.set(TalonSRXControlMode.Velocity, 1*Constants.position_units_per_meter/100);
      driveTrain.rightSpeedC.set(TalonSRXControlMode.Velocity, -1*Constants.position_units_per_meter/100);
      driveTrain.leftSpeedC.setNeutralMode(NeutralMode.Coast);
      driveTrain.rightSpeedC.setNeutralMode(NeutralMode.Coast);
      //driveTrain.leftSpeedC.set(TalonSRXControlMode.Position, time*1*Constants.position_units_per_meter);
      //driveTrain.rightSpeedC.set(TalonSRXControlMode.Position, -time*1*Constants.position_units_per_meter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    SmartDashboard.putString("going", "done");
    RobotContainer.inAuto = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return time>3;
  }
}
