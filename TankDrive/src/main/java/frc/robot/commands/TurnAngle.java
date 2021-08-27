// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;

public class TurnAngle extends CommandBase {
  private final DriveTrain driveTrain;

  private final double angle;

  /** Creates a new TankDrive. */
  public TurnAngle(DriveTrain dt, double a) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    
    angle = a;
    
    addRequirements(driveTrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   SmartDashboard.putBoolean("Turning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(NavXGyro.ahrs.getYaw()>angle){
        //turn right
        driveTrain.spin(-1*Constants.spin_rate);
    }
    else{
        //turn left
        driveTrain.spin(Constants.spin_rate);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
    SmartDashboard.putBoolean("Turning", false);
    
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean value =  (NavXGyro.ahrs.getYaw() <= angle + Constants.angle_error && NavXGyro.ahrs.getYaw() >= angle - Constants.angle_error);
    if(value) driveTrain.stop();
    return value;
  }
}
