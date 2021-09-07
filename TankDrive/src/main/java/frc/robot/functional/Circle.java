// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

import com.fasterxml.jackson.databind.util.ArrayBuilders.ShortBuilder;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class Circle extends Segment {
    public double[] center;
    public double radius;

    public Circle(double[] _center, double _radius, double[] _startPoint, double[] _endPoint){
        center = _center;
        radius = _radius;
        startPoint = _startPoint;
        endPoint = _endPoint;
        length = RobotContainer.getArcLength(this);
    }

    public Position getPosition(double distance){
        double angleDiff = distance/(2*Math.PI*radius) * 360;
        double startAngle = getAngle(startPoint);
        double endAngle = getAngle(endPoint);
        
        if(RobotContainer.shouldTurnLeft(startAngle, endAngle)){
            return new Position(radius*Math.cos(angleDiff + startAngle)+center[0], radius*Math.sin( startAngle + angleDiff)+center[1], (startAngle+angleDiff+90)%360);
        }
        return new Position(radius*Math.cos(startAngle - angleDiff)+center[0], radius*Math.sin(startAngle - angleDiff)+center[1], (startAngle-angleDiff-270)%360);
 

    }
    
    public double getAngle(double[] point){
        return Math.toDegrees(Math.atan2(point[1] - center[1], point[0] - center[0]));
    }
    public String toString(){
        return String.format("Circle: \n\tcenter: %f %f\n\tstart: %f %f\n\tend: %f %f\n\tlength: %f\n", center[0], center[1], 
        startPoint[0], startPoint[1], endPoint[0], endPoint[1], length);
    }
}

