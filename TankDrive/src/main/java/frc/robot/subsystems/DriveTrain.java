// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  
  public TalonSRX leftSpeedC = new TalonSRX(Constants.left_motor_port);
  public TalonSRX rightSpeedC = new TalonSRX(Constants.right_motor_port);
  
  //leftSpeedC = CTRE.Creator.createMasterTalon(Constants.leftSpeedC);
  public double currentLeft = 0;
  public double currentRight = 0;
  int loopIDX = 0;
   double kd = 0.05;
  public double kp = .001;
  double ki = 0;
  double kf = 1;
  /*private final Encoder m_leftEncoder 
      new Encoder(Constants.kLeftEncoderPort,
                  Constants.kLeftEncoderReversed);*/

  // The right-side drive encoder
  /*private final Encoder m_rightEncoder =
      new Encoder(Constants.kRightEncoderPort,Constants.kRightEncoderReversed);*/
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    
    
    leftSpeedC.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.PID_TYPE, 10);
    rightSpeedC.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.PID_TYPE, 10);
    leftSpeedC.configMotionAcceleration(Constants.max_acceleration*Constants.position_units_per_meter/10);
    rightSpeedC.configMotionAcceleration(Constants.max_acceleration*Constants.position_units_per_meter/10);
    leftSpeedC.setNeutralMode(NeutralMode.Brake);
    rightSpeedC.setNeutralMode(NeutralMode.Brake);
 
    leftSpeedC.config_kP(loopIDX, kp);
    leftSpeedC.config_kI(loopIDX, ki);
    leftSpeedC.config_kD(loopIDX, kd);
    leftSpeedC.config_kF(loopIDX, kf);
    rightSpeedC.config_kP(loopIDX, kp);
    rightSpeedC.config_kI(loopIDX, ki);
    rightSpeedC.config_kF(loopIDX, kf);
    rightSpeedC.config_kD(loopIDX, kd);
    //leftSpeedC.set(ControlMode.MotionMagic, demand);
  }

  public void setRightMotor(double speed){
    double value = speed*Constants.max_motor_percent;
    rightSpeedC.set(ControlMode.PercentOutput, value);
    currentRight = value;
  }
  public void setLeftMotor(double speed){
    double value = -speed*Constants.max_motor_percent;
    leftSpeedC.set(ControlMode.PercentOutput,value );
    currentLeft = -speed*Constants.max_motor_percent;
  }

  public void stop(){
  
    rightSpeedC.setNeutralMode(NeutralMode.Brake);
    leftSpeedC.setNeutralMode(NeutralMode.Brake);
    currentLeft = 0;
    currentRight = 0;
  }
  public void spin(double speed){
    //pos speed is left
    //neg speed is right
    rightSpeedC.set(ControlMode.PercentOutput, speed*Constants.max_motor_percent);
    currentLeft = speed;
    currentRight = speed;
    leftSpeedC.set(ControlMode.PercentOutput, speed*Constants.max_motor_percent);
  }
  public Pose2d getPose(){
    return new Pose2d(NavXGyro.ahrs.getDisplacementX(), NavXGyro.ahrs.getDisplacementY(), Rotation2d.fromDegrees(NavXGyro.ahrs.getYaw()));
  }
  public void tankDriveVolts(double vl, double vr){

  }
  /*public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftSpeedC.config_kP(loopIDX, kp);
    leftSpeedC.config_kI(loopIDX, ki);
    leftSpeedC.config_kD(loopIDX, kd);
    leftSpeedC.config_kF(loopIDX, kf);
    rightSpeedC.config_kP(loopIDX, kp);
    rightSpeedC.config_kI(loopIDX, ki);
    rightSpeedC.config_kF(loopIDX, kf);
    rightSpeedC.config_kD(loopIDX, kd);
    
  }
}
