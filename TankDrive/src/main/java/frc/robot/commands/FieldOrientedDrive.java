// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;

public class FieldOrientedDrive extends CommandBase {
  private final DriveTrain driveTrain;
  private final NavXGyro navx;
  double kp = 0.02;
  double ki = 0;
  double kd = 0.000;
  double kf = 0;
  double time = 0.02;
  double previous_error = 0;
  double error = 0;
  /** Creates a new TankDrive. */
  public FieldOrientedDrive(DriveTrain dt, NavXGyro n) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    navx = n;
    addRequirements(driveTrain);
    addRequirements(navx);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rightX = RobotContainer.xbox_controller.getRawAxis(Constants.right_x_axis);
    double rightY = RobotContainer.xbox_controller.getRawAxis(Constants.right_y_axis);
    double leftX = RobotContainer.xbox_controller.getRawAxis(Constants.left_x_axis);
    double leftY = RobotContainer.xbox_controller.getRawAxis(Constants.left_y_axis);
    
    double angle = Constants.stickTo360(rightX, rightY);
    SmartDashboard.putNumber("Field Oriented Angle", angle);
    SmartDashboard.putNumber("Navx Angle", Constants.navxTo360( NavXGyro.ahrs.getYaw()));
    double speed = Math.sqrt(Math.pow(rightX, 2)+Math.pow(rightY, 2));
    
    error = Constants.angleDistance(angle);
      double angle_corrector = Math.max(kp*error + ki*error*time + kd*(error-previous_error)/time, 1);
      speed*=Constants.max_motor_percent;
      if (leftX != 0) {
        driveTrain.spin(leftX);
        return;
      }
      SmartDashboard.putNumber("Angle Distance", Constants.angleDistance(angle));
      SmartDashboard.putBoolean("Should go Back",Constants.angleDistance(angle) > 90 );
      if (Constants.angleDistance(angle) > 90) {
        speed *= -1;
        if(Constants.shouldTurnLeft(NavXGyro.ahrs.getYaw(), (angle - 180 + 360)%360)){
          //turn left
          driveTrain.setLeftMotor(speed/angle_corrector);
          driveTrain.setRightMotor(speed);
        }
        else{
          //turn right
          driveTrain.setLeftMotor(speed);
          driveTrain.setRightMotor(speed/angle_corrector);
        }
      }
      else {
        if(Constants.shouldTurnLeft(NavXGyro.ahrs.getYaw(), angle)){
          //turn left
          driveTrain.setLeftMotor(speed/angle_corrector);
          driveTrain.setRightMotor(speed);
        }
        else{
          //turn right
          driveTrain.setLeftMotor(speed);
          driveTrain.setRightMotor(speed/angle_corrector);
        }
      }
      previous_error = error;

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
