// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
;

public class FollowContPath extends CommandBase {
  /** Creates a new FollowContPath. */
  double initialTime;
  public FollowContPath(int[] start, int[] end, int[][] points, double maxV, double maxA) {
    // Use addRequirements() here to declare subsystem dependencies.
    TrajectoryConfig config = new TrajectoryConfig(maxV, maxA);
    Pose2d startPos = new Pose2d(start[0], start[1], Rotation2d.fromDegrees(0));
    Pose2d endPos = new Pose2d(end[0], end[1], Rotation2d.fromDegrees(0));
    ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
    for(int[] point: points)
      interiorWaypoints.add(new Translation2d(point[0], point[1]));

    var trajectory = TrajectoryGenerator.generateTrajectory(
      startPos, 
      interiorWaypoints, 
      endPos, 
      config);
      initialTime = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
