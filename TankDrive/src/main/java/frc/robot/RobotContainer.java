// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.kauailabs.navx.frc.AHRS;



import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.DisplayMPX;

import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TurnAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavXGyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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



  //Buttons
  Button aButtonTurn90Left = new JoystickButton(xbox_controller, Constants.a_button_num);
  Button bButtonTurn90Right = new JoystickButton(xbox_controller, Constants.b_button_num);
  Button xButtonSwitchDrive = new JoystickButton(xbox_controller, Constants.x_button_num);
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
    
    
    
    
    
    
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    aButtonTurn90Left.whenPressed(new TurnAngle(driveTrain, 90));
    bButtonTurn90Right.whenPressed(new TurnAngle(driveTrain,  -90));
    xButtonSwitchDrive.whenPressed(new SwitchDriveMode(driveTrain, navx));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
