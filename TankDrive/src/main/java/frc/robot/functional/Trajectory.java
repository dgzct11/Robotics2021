// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

/** Add your docs here. */
public class Trajectory {
    int[][] points;
    int[] distances;
    public Trajectory(int[][] points, int[] distances){
        this.points = points;
        this.distances = distances;
    }

}
