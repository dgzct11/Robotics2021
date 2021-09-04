// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class Trajectory {
    double[][] points;
    double[] distances;
    Line[] lines;
    Circle[] circles;

    double maxVelocity;
    double acceleration;
    double totalDistance = 0;
    public Trajectory(double[][] points, double[] distances){
        this.points = points;
        this.distances = distances;
        initializeSegments();
        getTotalDistance();
        
    }

    public void getTotalDistance(){
        for(Line line: lines){
            totalDistance += RobotContainer.distance(line.startPoint, line.endPoint);
        }

        for( Circle circle: circles){
            totalDistance += RobotContainer.getArcLength(circle);
        }
       
    }
    public void initializeSegments(){
        lines = new Line[points.length-1];
        circles = new Circle[points.length - 2];
        int l = 0;
        int c = 0;
        double[] startPoint = points[0];
        double secondSlope = 0;
        for(int i = 0; i<points.length-2; i++){
            //establish points
            
            double[] cornerPoint = points[i+1];
            double[] nextCorner = points[i+2];
            double distance = RobotContainer.distance(startPoint, cornerPoint);
            double ratio = (distance - distances[i])/distance;
            double[] circleStart = { (cornerPoint[0]-startPoint[0])*ratio + startPoint[0], (cornerPoint[1]-startPoint[1])*ratio + startPoint[1] };

            double distance2 = RobotContainer.distance(nextCorner, cornerPoint);
            double ratio2 = (distance2 - distances[i])/distance2;
            double[] circleEnd = { (nextCorner[0]-cornerPoint[0])*ratio2 + cornerPoint[0], (nextCorner[1]-cornerPoint[1])*ratio2 + cornerPoint[1] };

            //find circle equation
            double firstSlope = (startPoint[1]-cornerPoint[1])/(startPoint[0]-cornerPoint[0]);
            double firstPerpendicular = -1/firstSlope;
            secondSlope = (nextCorner[1]-cornerPoint[1])/(nextCorner[0]-cornerPoint[0]);
            double secondPerpendicular = -1/secondSlope;
            Line line1 = new Line(circleStart , firstPerpendicular);
            Line line2 = new Line(circleEnd,  secondPerpendicular);
            double[] center = line1.getIntersection(line2);
            double radius = RobotContainer.distance(center, startPoint);
            Circle circle = new Circle(center, radius, circleStart, circleEnd);
            Line line = new Line(startPoint, circleStart);
            lines[l] = line;
            l++;
            circles[c] = circle;
            c++;
            startPoint = circleEnd;
        }
        lines[lines.length-1] = new Line(startPoint, points[-1]);



    }

}
