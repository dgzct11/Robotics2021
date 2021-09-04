// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

/** Add your docs here. */
public class Circle {
    public double[] center;
    public double radius;
    public double[] startPoint, endPoint;
    public Circle(double[] _center, double _radius, double[] _startPoint, double[] _endPoint){
        center = _center;
        radius = _radius;
        startPoint = _startPoint;
        endPoint = _endPoint;
    }

    
}
