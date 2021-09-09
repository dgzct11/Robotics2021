// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.functional.Position;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;
import frc.robot.subsystems.Odometry;

public class DisplayMPX extends CommandBase {
  /** Creates a new DisplayMPX. */
  private AHRS ahrs;
  private NavXGyro navx;
  Odometry odometry;
  DriveTrain driveTrain;
  public DisplayMPX(NavXGyro n, Odometry od, DriveTrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    ahrs = NavXGyro.ahrs;
    navx = n;
    odometry = od;
    driveTrain = dt;
    addRequirements(navx);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber(   "IMU_Yaw",              RobotContainer.navxTo360(ahrs.getYaw()));
 

    
    SmartDashboard.putNumber(   "Velocity_X",           ahrs.getVelocityX());
    SmartDashboard.putNumber(   "Velocity_Y",           ahrs.getVelocityY());
    SmartDashboard.putNumber(   "Displacement_X",       ahrs.getDisplacementX());
    SmartDashboard.putNumber(   "Displacement_Y",       ahrs.getDisplacementY());
    
    Position currentPosition = odometry.getPosition();
    SmartDashboard.putNumber("Odometry x: ", currentPosition.x );
    SmartDashboard.putNumber("Odometry y: ", currentPosition.y);
    SmartDashboard.putNumber("Odometry angle ", currentPosition.angle);
    SmartDashboard.putNumber("Left Motor Position", driveTrain.leftSpeedC.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Motor Position", driveTrain.rightSpeedC.getSelectedSensorPosition());

    SmartDashboard.putNumber("Drive Mode: ", Constants.driveMode);
  
    SmartDashboard.putNumber("Max Motor Percent", Constants.max_motor_percent);
    SmartDashboard.putNumber("Left Position", driveTrain.leftSpeedC.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Position", driveTrain.leftSpeedC.getSelectedSensorPosition());
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
