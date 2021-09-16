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
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Odometry;

public class FollowTrajectory extends CommandBase {
  /** Creates a new FollowTrajectory. */
  DriveTrain driveTrain;
  Odometry odometry;
  Trajectory trajectory;
  double initialTime;
  double previousTime;
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
    previousTime = System.currentTimeMillis()/1000;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //find currentPosition
    //find the position of the robot after some time interval
    //
    odometry.updatePosition();
    Position currentPosition = odometry.getPosition();
    double timeUnit = System.currentTimeMillis()/1000 - previousTime;
    Position newPos = trajectory.getPosition((System.currentTimeMillis()-initialTime)/1000+timeUnit);
    
    double[] currentPosXY = {currentPosition.x, currentPosition.y};
    double[] newPosXY = {newPos.x, newPos.y};
    Line lineStart = new Line(currentPosXY, Math.tan(Math.toRadians(currentPosition.angle)));
    Line lineEnd = new Line(newPosXY, Math.tan(Math.toRadians(newPos.angle)));
    double[] center = lineStart.getIntersection(lineEnd);
    if(currentPosition.angle == newPos.angle){
      driveTrain.leftSpeedC.set(TalonSRXControlMode.Velocity,  1*Constants.position_units_per_meter/timeUnit/1000);
      driveTrain.rightSpeedC.set(TalonSRXControlMode.Velocity, -1*Constants.position_units_per_meter/timeUnit/1000);
      return;
    }
    double radius = RobotContainer.distance(center, currentPosXY);
    double theta = 2*Math.asin(RobotContainer.distance(currentPosXY, newPosXY)/2/radius);
    double outer = theta*(radius+Constants.distance_between_wheels/2);
    double inner = theta*(radius - Constants.distance_between_wheels/2);
    if(RobotContainer.shouldTurnLeft(currentPosition.angle, newPos.angle)){
      driveTrain.leftSpeedC.set(TalonSRXControlMode.Velocity,  inner*Constants.position_units_per_meter/timeUnit/10);
      driveTrain.rightSpeedC.set(TalonSRXControlMode.Velocity,  -outer*Constants.position_units_per_meter/timeUnit/10);
    }
    else{
      driveTrain.leftSpeedC.set(TalonSRXControlMode.Velocity, outer * Constants.position_units_per_meter/timeUnit/10);
      driveTrain.rightSpeedC.set(TalonSRXControlMode.Velocity, -inner * Constants.position_units_per_meter/timeUnit/10);
      }
    previousTime = System.currentTimeMillis()/1000;
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
