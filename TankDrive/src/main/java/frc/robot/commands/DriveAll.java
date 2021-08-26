// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.NavXGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAll extends ParallelCommandGroup {
  /** Creates a new DriveAll. */
  private final DriveTrain driveTrain;
  private final XboxController xbox;
  private final NavXGyro navx;

  public DriveAll(DriveTrain dt, NavXGyro n, XboxController x) {
    driveTrain = dt;
    navx = n;
    xbox = x;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TurnAngle(driveTrain, navx, 90, xbox));
    addCommands(new TankDrive(driveTrain));
    addCommands(new DisplayMPX(n));
  }
}
