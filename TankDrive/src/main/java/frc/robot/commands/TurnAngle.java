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

public class TurnAngle extends CommandBase {
  private final DriveTrain driveTrain;
  
  private final double angle;
  double kp = 0.04;
  double ki = 0.0;
  double kd = 0.0001;
  double kf = 0.00;
  double kTolerance = 2.0;
  double previous_error = 0;
  double error = 0;
  double time = 0.05;
  double previousTime = 0;
  
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
    previous_error = RobotContainer.angleDistance(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("time", time);
    error = RobotContainer.angleDistance(angle);
    double p_output = kp*error + ki*error*time + kd*(error-previous_error)/time;
    if(p_output>0) p_output = Math.min(p_output, Constants.max_motor_percent);
    else p_output = Math.max(p_output, -Constants.max_motor_percent);
    SmartDashboard.putNumber("PID Error", p_output);
    if(RobotContainer.shouldTurnLeft(NavXGyro.ahrs.getAngle(), angle)){
        //turn right
        driveTrain.spin(-1*p_output);
    }
    else{
        //turn left
        driveTrain.spin(p_output);
    }
    previous_error = error;
  
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
    boolean value = RobotContainer.currentAngleEquals(angle);  
    if(value) driveTrain.stop();
    return value;
  }
}
