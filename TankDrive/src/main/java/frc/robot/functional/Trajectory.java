// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.functional;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.ArrayList;
/** Add your docs here. */
public class Trajectory {
    double[][] points;
    double[] distances;
   
    ArrayList<Segment> segments;
    double maxVelocity;
    double acceleration;
    double breakAcceleration;
    double totalDistance = 0;
    double currentDistance = 0;
    double currentVelocity = 0;
    int currentIndex = 0;
    double accelerationDistance = 0;
    double breakingDistance = 0;
    double timeToMax = 0;
    double timeToBreak = 0;
    double currentLength = 0;
    public Trajectory(double[][] points, double[] distances){
        this.points = points;
        this.distances = distances;
        initializeSegments();
        getTotalDistance();
        
    }
    public Position getPosition(double time){
        /*
        if currentDistance < acceldistance
        update velocity
        update currentDistance
        

        if current Distance > totalDistance - breaking distance
        update velocity
        update currentPosition
        

        else
        update current Position

        if currentPosition>segment.endPosiont, go to next segment. 

        calculate position
        return position
        */
        if(currentDistance > totalDistance){
            Segment seg = segments.get(segments.size()-1);
            return new Position(seg.endPoint, RobotContainer.angleFromSlope(seg.startPoint, seg.endPoint));
        }
        if(currentDistance < accelerationDistance){
            //xf  = a/2t^2
            currentDistance = acceleration/2*time*time;
        }
        else if(currentDistance > breakingDistance){
            // xf = x + vt -a/2^t
            currentDistance = breakingDistance + maxVelocity*(time-timeToBreak) - breakAcceleration/2*(time-timeToBreak)*(time-timeToBreak);
        }
        else{
            // xf = a/2t^2
            //t = sqrt(2x/a)
            currentDistance += (time-timeToMax)*maxVelocity;
        }

        if(currentDistance > currentLength ){
            currentIndex += 1;
            currentLength += segments.get(currentIndex).length;
        }

        //turn currentDistance into position
        return segments.get(currentIndex).getPosition(currentDistance-(currentLength-segments.get(currentIndex).length));
        
    }
    
    public void getTotalDistance(){
        for(Segment seg: segments){
           totalDistance += seg.length;
        }
       accelerationDistance = (maxVelocity*maxVelocity)/(2*acceleration);
       breakingDistance = totalDistance - (maxVelocity*maxVelocity)/(2*breakAcceleration);
       timeToMax = Math.sqrt(2*accelerationDistance/acceleration);
       timeToBreak = timeToMax + (breakingDistance-accelerationDistance)/maxVelocity;
       currentLength = segments.get(0).length;
    }
    public void initializeSegments(){
        segments = new ArrayList<Segment>();
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
            segments.add(line);
            segments.add(circle);
            startPoint = circleEnd;
        }
        segments.add(new Line(startPoint, points[-1]));



    }

}
