// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.functional.Position;

public class Odometry extends SubsystemBase {
  /** Creates a new Odometry. */
  private Position currentPosition;
  public DriveTrain driveTrain;
  double previousPosLeft = 0;
  double previousPosRight = 0;

  public Odometry(DriveTrain dt) {
    driveTrain = dt;
    currentPosition = new Position(0, 0, NavXGyro.getAngle());
    driveTrain.leftSpeedC.setSelectedSensorPosition(0);
    driveTrain.rightSpeedC.setSelectedSensorPosition(0);
  }

  public void reset(){
    currentPosition = new Position(0, 0, 0);
  }
  public void updatePosition(){
    //figure out change in angle
    double currentPosLeft = driveTrain.leftSpeedC.getSelectedSensorPosition()/Constants.position_units_per_meter;
    double currentPosRight = -driveTrain.rightSpeedC.getSelectedSensorPosition()/Constants.position_units_per_meter;
    double angleDifference = Math.toDegrees(Math.atan2(currentPosRight-previousPosRight-currentPosLeft+previousPosLeft, Constants.distance_between_wheels));
    
    //

    //add current position
    double posChangeAvarage = (currentPosLeft-previousPosLeft + currentPosRight-previousPosRight)/2;
    double dx = Math.sin(Math.toRadians(currentPosition.angle))*posChangeAvarage;
    double dy = Math.cos(Math.toRadians(currentPosition.angle))*posChangeAvarage;

    previousPosLeft = currentPosLeft;
    previousPosRight = currentPosRight;
    //update current position
    currentPosition.add(dx, dy);
    currentPosition.addAngle(angleDifference);

    

  }
  public Position getPosition(){
    return currentPosition;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}