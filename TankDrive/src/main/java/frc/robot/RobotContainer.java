// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.PIDController;

//import org.graalvm.compiler.nodes.ConstantNode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.AutonomusCommands;
import frc.robot.commands.ChangeMaxSpeed;
import frc.robot.commands.DisplayMPX;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnAngle;
import frc.robot.functional.Circle;
import frc.robot.functional.Line;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavXGyro;
//import jdk.vm.ci.meta.Constant;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //devices
   public  static XboxController xbox_controller = new XboxController(Constants.xbox_port);
 
  //subsystems
  private final NavXGyro navx;
  private final DriveTrain driveTrain;

  //commands
  private final TankDrive tankDrive;
  private final DisplayMPX displayMPX;
  private final TurnAngle tn;
  
  public static boolean inAuto = false;

  //Buttons
  /*Button aButtonTurn90Left = new JoystickButton(xbox_controller, Constants.a_button_num);
  Button bButtonTurn90Right = new JoystickButton(xbox_controller, Constants.b_button_num);
  Button xButtonSwitchDrive = new JoystickButton(xbox_controller, Constants.x_button_num);
  Button yButtonGo0 = new JoystickButton(xbox_controller, Constants.y_button_num);*/
  Button leftPad = new POVButton(xbox_controller, Constants.left_pad_num);
  Button rightPad = new POVButton(xbox_controller, Constants.right_pad_num);
  Button upPad = new POVButton(xbox_controller, Constants.up_pad_num);
  Button downPad = new POVButton(xbox_controller, Constants.down_pad_num);

  Button xButtonSwitchDrive = new JoystickButton(xbox_controller, Constants.x_button_num);

  Button rightButtonIncMotor = new JoystickButton(xbox_controller, Constants.rb_button_num);
  Button leftButtonDecMotor = new JoystickButton(xbox_controller, Constants.lb_button_num);
  DriveStraightDistance driveStraightDistance;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    //subsytems 
    driveTrain = new DriveTrain();
    navx = new NavXGyro();
    
    //commands
    tankDrive = new TankDrive(driveTrain);
    tankDrive.addRequirements(driveTrain);
  
    tn = new TurnAngle(driveTrain, 90);
    tn.addRequirements(navx);
    tn.addRequirements(driveTrain);
    displayMPX = new DisplayMPX(navx);
    displayMPX.addRequirements(navx);
   
    driveTrain.setDefaultCommand(tankDrive);
    navx.setDefaultCommand(displayMPX);
    configureButtonBindings();
    driveStraightDistance = new DriveStraightDistance(0.5, 0, driveTrain);
    
    
    
    
    
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    leftPad.whenPressed(new TurnAngle(driveTrain, 90));
    rightPad.whenPressed(new TurnAngle(driveTrain,  270));
    upPad.whenPressed(new TurnAngle(driveTrain, 0));
    downPad.whenPressed(new TurnAngle(driveTrain, 180));
    xButtonSwitchDrive.whenPressed(new SwitchDriveMode(driveTrain, navx));
    rightButtonIncMotor.whenPressed(new ChangeMaxSpeed(0.1));
    leftButtonDecMotor.whenPressed(new ChangeMaxSpeed(-0.1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutonomusCommands(driveTrain);
  }

  public static double navxTo360(double angle){
        
    if (angle<=0) angle += 360;

    return 360-angle;
  }
  public static double to360(double angle) {
    if (angle <= 0) angle += 360;

    return Math.abs(angle%360);
  }
  public static double stickTo360(double x, double y){
   return (to360(Math.toDegrees(Math.atan2(-y,x)))+270)%360;
  }
  public static boolean shouldTurnLeftNavx(double currentNavxAngle, double targetAngle){
    double angle = navxTo360(currentNavxAngle);
    boolean value = false;

    if(targetAngle < 180) value = angle<targetAngle || angle> 180+targetAngle;
    else value = angle<targetAngle && angle> targetAngle-180;
    return value;
  }
  public static boolean shouldTurnLeft(double currentNavxAngle, double targetAngle){
    double angle = currentNavxAngle;
    boolean value = false;

    if(targetAngle < 180) value = angle<targetAngle || angle> 180+targetAngle;
    else value = angle<targetAngle && angle> targetAngle-180;
    return value;
  }
  public static boolean currentAngleEquals(double angle){
    double currentAngle = navxTo360(NavXGyro.ahrs.getYaw());
    return ( shouldTurnLeft(currentAngle, angle+Constants.angle_error) ^ shouldTurnLeft(currentAngle, angle-Constants.angle_error) ) && !(shouldTurnLeft((currentAngle+90)%360,angle));
  }
  public static double angleDistance(double targetAngle){
    double angle = navxTo360(NavXGyro.ahrs.getYaw());
    double distance = Math.abs(targetAngle - angle)%360;
    if (distance > 180) distance = 360 - distance;
    return distance;
  }

  public static double distance(double[] p1, double[] p2){
    return Math.sqrt( Math.pow(p1[1] - p2[1], 2) + Math.pow(p1[0] - p2[0], 2));
  }

  public static double getArcLength(Circle circle){
    Line base = new Line(circle.startPoint, circle.endPoint);
    double[] midPoint = base.getMidPoint();
    double halfAngle = Math.atan(distance(midPoint, base.startPoint)/distance(midPoint, circle.center));
    return halfAngle*2*circle.radius;
  }

  public static double angleFromSlope(double[] start, double[] end){
    return Math.toDegrees(Math.atan2((end[1] - start[1]), end[0] - start[0]));
  }
/*
  public Command getPathFollowCommand(){
   

    // Create a voltage constraint to ensure we don't accelerate too fast
     var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                 Constants.ksVolts,
                Constants.kvVoltSecondPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
             Constants.kDriveKinematics,
            10);
 
    // Create config for trajectory
     TrajectoryConfig config =
         new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                      Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
   Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
            
    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            driveTrain::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.ksVolts,
                Constants.kvVoltSecondPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter),
           Constants.kDriveKinematics,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveTrain::tankDriveVolts,
            driveTrain);

    // Reset odometry to the starting pose of the trajectory.
   //driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));

  }
  */
}
