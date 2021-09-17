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
import frc.robot.subsystems.NavXGyro;
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
    odometry.reset();
    NavXGyro.ahrs.reset();;
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
    double[] speeds = getSpeed(currentPosXY, newPosXY, currentPosition.angle);
    
      driveTrain.leftSpeedC.set(TalonSRXControlMode.Velocity, speeds[0] * Constants.position_units_per_meter/timeUnit/10);
      driveTrain.rightSpeedC.set(TalonSRXControlMode.Velocity, speeds[1] * Constants.position_units_per_meter/timeUnit/10);
      
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

  public static double[] getSpeed(double[] start, double[] end, double angle){
    //returns left right
    double m = RobotContainer.slopeFromAngle(angle);
    double x,y;
    double a = 2*end[0] - 2*start[0];
    double b = 2*end[1] - 2*start[1];
    double speed = 0;
    double c = end[0] * end[0] - start[0]*start[0] + end[1]*end[1] - start[1]*start[1];
    
    if(Double.isInfinite(m)){
        //if the slope is infinity, than you know
        // the y value, or k.
        y = start[1];

        x = (c-b*y) / a;
        System.out.println(x+" "+y);
    }
    else if(m == 0){
        //you know the x value, or h
        x = start[0];
        y = (c - a*x)/b;
        
    }
    else {
        x = (c + b*m*start[0] - b*start[1])/(a + b*m);
        y = m*(x-start[0]) + start[1];
       
    }
    
    if(Double.isInfinite(x) || Double.isInfinite(y)){
        speed = RobotContainer.distance(start, end);
        double[] result = {speed, speed};
        return result;
    }
    else{
        double center[] = {x, y};
        double radius = RobotContainer.distance(center, start);
        double theta = 2*Math.asin(RobotContainer.distance(start, end)/(2*radius));
        
        double outer = (radius + 1) * theta;
        double inner = (radius - 1) * theta;
        if(RobotContainer.shouldTurnLeft(angle, RobotContainer.angleToPoint(start, end))){
            double[] result = {inner, outer};
            return result;
        }
        double[] result = {outer, inner};
        return result;
    }
}
}
