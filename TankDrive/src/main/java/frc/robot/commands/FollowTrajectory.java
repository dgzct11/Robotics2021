// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Odometry;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */
  DriveTrain driveTrain;
  Odometry odometry;
  public FollowTrajectory(DriveTrain dt, Odometry od) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    odometry = od;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.inAuto = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.inAuto = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
