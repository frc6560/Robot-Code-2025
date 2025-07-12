package com.team6560.frc2025.utility;

import java.util.ArrayList;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** A method to generate a path to a pose of our choice! */
public class PathCalculator {
    public ArrayList<Pose2d> controlPointList = new ArrayList<Pose2d>();

    public ArrayList<Pose2d> straightNavigationPoses = new ArrayList<Pose2d>();
    public ArrayList<Pose2d> controlPointNavigationPoses = new ArrayList<Pose2d>();

    public final double CONTROL_POINT_DISTANCE = 2.0;
    public final double FIRST_CONTROL_HEADING = 1.0;
    public final double SECOND_CONTROL_HEADING; // this can be tuned.

    public PathCalculator(double secondControlHeading){
        this.SECOND_CONTROL_HEADING = secondControlHeading;
        Pose2d[] navigationTags = {
            new Pose2d(13.89, 4.02, Rotation2d.fromDegrees(0)),
            new Pose2d(12.64, 3.30, Rotation2d.fromDegrees(300)),
            new Pose2d(12.64, 4.74, Rotation2d.fromDegrees(60)),
            new Pose2d(3.65, 4.02, Rotation2d.fromDegrees(180)),
            new Pose2d(4.90, 4.74, Rotation2d.fromDegrees(120)),
            new Pose2d(4.90, 3.30, Rotation2d.fromDegrees(240)),
        };
        for (Pose2d tag : navigationTags) {
            controlPointList.add(new Pose2d(tag.getX() + CONTROL_POINT_DISTANCE * Math.cos(tag.getRotation().getRadians()), tag.getY() + CONTROL_POINT_DISTANCE * Math.sin(tag.getRotation().getRadians()), Rotation2d.fromDegrees(FIRST_CONTROL_HEADING)));
        }

        
    }
}
