// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

/** Add your docs here. */
public class Position {
    public double x, y, angle;
    public Position(double _x, double _y, double _angle){
        angle = _angle;
        x = _x;
        y = _y;
    }
    public Position(double[] pos, double _angle){
        x = pos[0];
        y = pos[1];
        angle = _angle;
    }

    public void addAngle(double a){
        angle = (angle+a+360)%360;
    }
    public void add(double dx, double dy){
        x += dx;
        y += dy;
    }
}
