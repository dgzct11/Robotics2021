// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;

public class TankDriveStraight extends CommandBase {
  private final DriveTrain driveTrain;
  private final NavXGyro navx;
  private double angle;
  /** Creates a new TankDrive. */
  public TankDriveStraight(DriveTrain dt, NavXGyro n) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    navx = n;
    addRequirements(driveTrain);
    addRequirements(navx);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      angle = navx.ahrs.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = RobotContainer.xbox_controller.getRawAxis(Constants.left_y_axis);
    double rightY = RobotContainer.xbox_controller.getRawAxis(Constants.right_y_axis);
    if(Math.abs(leftY)>Constants.min_joystick_correction_threshold && Math.abs(rightY)>Constants.min_joystick_correction_threshold){
        if(navx.ahrs.getAngle() < angle){
            //turn left
            driveTrain.setLeftMotor(driveTrain.currentLeft*Constants.angle_correction_multiplier);
            driveTrain.setRightMotor(driveTrain.currentRight/Constants.angle_correction_multiplier);
        }
        else{
            //turn right
            driveTrain.setLeftMotor(driveTrain.currentLeft/Constants.angle_correction_multiplier);
            driveTrain.setRightMotor(driveTrain.currentRight*Constants.angle_correction_multiplier);
        }
    }
    else{
        driveTrain.setRightMotor(rightY);
        driveTrain.setLeftMotor(leftY);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
