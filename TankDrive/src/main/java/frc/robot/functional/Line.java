// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

/** Add your docs here. */
public class Line {
    public double x, y, slope;
    public Line(double _x, double _y, double _slope){
        x = _x;
        y = _y;
        slope = _slope;

    }

    public double[] getIntersection(Line line){
        
        double[] result = new double[2];
        
        result[0] = (-line.slope * line.x + line.y - y + slope * x)/(slope - line.slope);
        result[1] = slope * (result[0] - x) + y;
        return result;
    }
}
