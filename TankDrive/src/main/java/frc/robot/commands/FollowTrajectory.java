// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.functional.Line;
import frc.robot.functional.Position;
import frc.robot.functional.Trajectory;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Odometry;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */
  DriveTrain driveTrain;
  Odometry odometry;
  Trajectory trajectory;
  double initialTime;
  
  public FollowTrajectory(DriveTrain dt, Odometry od, double[][] points, double[] distances) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = dt;
    odometry = od;
    trajectory = new Trajectory(points,distances,.1,.1);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.inAuto = true;
    initialTime = System.currentTimeMillis(); 
    driveTrain.leftSpeedC.setSelectedSensorPosition(0);
    driveTrain.rightSpeedC.setSelectedSensorPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //find currentPosition
    //find the position of the robot after some time interval
    //
    odometry.updatePosition();
    Position currentPosition = odometry.getPosition();
    Position newPos = trajectory.getPosition((System.currentTimeMillis()-initialTime)/1000+0.01);
    Line lineStart = new Line();
    
  }

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
