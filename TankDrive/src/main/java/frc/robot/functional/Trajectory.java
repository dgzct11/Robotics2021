// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Trajectory {
    double[][] points;
    double[] distances;
    Line[] lines;
    Circle[] circles;
    public Trajectory(double[][] points, double[] distances){
        this.points = points;
        this.distances = distances;
        lines = new Line[points.length-1];
        circles = new Circle[points.length - 2];
        int l = 0;
        int c = 0;
        for(int i = 0; i<points.length-2; i++){
            //establish points
            double[] initialPoint = points[i];
            double[] cornerPoint = points[i+1];
            double[] finalPoint = points[i+2];
            double distance = Math.sqrt(Math.pow(initialPoint[0]-cornerPoint[0], 2) + Math.pow(initialPoint[1]-cornerPoint[1] , 2));
            double ratio = (distance - distances[i])/distance;
            double[] startPoint = { (cornerPoint[0]-initialPoint[0])*ratio + cornerPoint[0], (cornerPoint[1]-initialPoint[1])*ratio + cornerPoint[1] };

            double distance2 = Math.sqrt(Math.pow(finalPoint[0]-cornerPoint[0], 2) + Math.pow(finalPoint[1]-cornerPoint[1], 2));
            double ratio2 = (distance2 - distances[i])/distance;
            double[] endPoint = { (finalPoint[0]-cornerPoint[0])*ratio2 + cornerPoint[0], (cornerPoint[1]-initialPoint[1])*ratio2 + cornerPoint[1] };

            //find circle equation
            double firstSlope = (initialPoint[1]-cornerPoint[1])/(initialPoint[0]-cornerPoint[0]);
            double firstPerpendicular = -1/firstSlope;
            double secondSlope = (finalPoint[1]-cornerPoint[1])/(finalPoint[0]-cornerPoint[0]);
            double secondPerpendicular = -1/secondSlope;
            Line line1 = new Line(startPoint[0], startPoint[1], firstPerpendicular);
            Line line2 = new Line(endPoint[0], endPoint[1], secondPerpendicular);
            double[] center = line1.getIntersection(line2);
            double radius = RobotContainer.distance(center, startPoint);
            Circle circle = new Circle(center, radius, startPoint, endPoint);
            lines[l] = line1;
            lines[l+1] = line2;
            l+=2;
            circles[c] = circle;
            c++;
        }


    }

}
