// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavXGyro;

public class TurnAngle extends CommandBase {
  private final DriveTrain driveTrain;
  private final NavXGyro navx;
  private final double angle;
  private XboxController xbox;
  /** Creates a new TankDrive. */
  public TurnAngle(DriveTrain dt, NavXGyro n, double a, XboxController x) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    navx = n;
    angle = a;
    xbox = x;
    addRequirements(driveTrain);
    addRequirements(navx);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Constants.turned) this.end(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xbox.getAButtonPressed()){
    if(navx.ahrs.getAngle()>angle){
        //turn right
        driveTrain.spin(-1*Constants.spin_rate);
    }
    else{
        //turn left
        driveTrain.spin(Constants.spin_rate);
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setLeftMotor(0);
    driveTrain.setRightMotor(0);
    Constants.turned = True;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return navx.ahrs.getAngle() >= angle + Constants.angle_error && navx.ahrs.getAngle() <= angle - Constants.angle_error;
  }
}
