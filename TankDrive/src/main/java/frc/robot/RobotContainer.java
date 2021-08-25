// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.kauailabs.navx.frc.AHRS;



import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DisplayMPX;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.NavXGyro;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public  static XboxController xbox_controller = new XboxController(Constants.xbox_port);

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final DriveTrain driveTrain;
  private final TankDrive tankDrive;
  private final AHRS ahrs;
  private final NavXGyro navx;
  private final DisplayMPX displayMPX;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driveTrain = new DriveTrain();
    tankDrive = new TankDrive(driveTrain);
    tankDrive.addRequirements(driveTrain);
    driveTrain.setDefaultCommand(tankDrive);
    configureButtonBindings();

    navx = new NavXGyro();
    
    ahrs = new AHRS(Constants.mxp_port);
    displayMPX = new DisplayMPX(ahrs, navx);
    displayMPX.addRequirements(navx);
    navx.setDefaultCommand(displayMPX);
    
  }

  /*
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
