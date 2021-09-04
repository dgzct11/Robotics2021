// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

import frc.robot.RobotContainer;

/** Add your docs here. */
public class Line {
    public double slope;
    public double[] startPoint, endPoint;
  
    public Line(double[] start, double[] end){
        startPoint = start;
        endPoint = end;
        slope = (start[1] - end[1]) / (start[0]-end[0]);
        

    }
    public Line(double[] end, double s){
        endPoint = end;
        slope = s;
    }

    public double[] getIntersection(Line line){
        
        double[] result = new double[2];
        
        result[0] = (-line.slope * line.endPoint[0] + line.endPoint[1] - endPoint[1] + slope * endPoint[0])/(slope - line.slope);
        result[1] = slope * (result[0] - endPoint[0]) + endPoint[1];
        return result;
    }

    public double[] getMidPoint(){
        double[] result = new double[2];
        result[0] = (endPoint[0] - startPoint[0])/2 + startPoint[0]; 
        result[1] = (endPoint[1] - startPoint[1])/2 + startPoint[1];
        return result; 
    }
}
