package com.team6560.frc2025.utility;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// TODO LIST:
// 1. hardbake the target poses in RobotContainer
// 2. write the path generation code
// 

/** A method to generate a smooth path to a pose of our choice. */
public class PathCalculator {
    private final Map<Double, Pose2d> setpointMapRed = new HashMap<Double, Pose2d>();
    private final Map<Double, Pose2d> setpointMapBlue = new HashMap<Double, Pose2d>();

    public int currentSegment;

    // Constants
    public final double DISTANCE_TO_REEF = 2.0; // distance of each setpoint to the reef. 

    // these values might need to be tuned.
    public final double FIRST_CONTROL_HEADING_INITIAL = 1.15; 
    public final double FIRST_CONTROL_HEADING_FINAL = 1.15; 
    public final double SECOND_CONTROL_HEADING_INITIAL = 1.0;
    public final double SECOND_CONTROL_HEADING_FINAL = 1.0; 

    public PathCalculator(Pose2d currentPose){
        // List of all waypoints
        Pose2d[] setpointsBlue = {
            // arrange them in particular order. this is important. start from 
            new Pose2d(3.65, 4.02, Rotation2d.fromDegrees(180)),
            new Pose2d(4.90, 4.74, Rotation2d.fromDegrees(120)),
            new Pose2d(4.90, 3.30, Rotation2d.fromDegrees(240)),
            // continue
        };
        Pose2d[] setpointsRed = {
            // these are wrong, fix
            new Pose2d(3.65, 4.02, Rotation2d.fromDegrees(0)),
            new Pose2d(4.90, 4.74, Rotation2d.fromDegrees(60)),
            new Pose2d(4.90, 3.30, Rotation2d.fromDegrees(-60)),
            // continue making more
        };

        // Populate the setpoint maps based on alliance
        for (int i = 0; i < setpointsBlue.length; i++) {
            Pose2d blueTag = setpointsBlue[i];
            Pose2d redTag = setpointsRed[i];
            setpointMapBlue.put((double) (i + 1) / 2, blueTag);
            setpointMapRed.put((double) (i + 1) / 2, redTag);
        }

    }

    /** Gets a control point based upon current pose, magnitude, and direction
     * @param currentPose A point of the curve
     * @param magnitude The distance to the control point
     * @param direction The direction of the control point, in radians
     */
    public Pose2d getPoseDirectionFrom(Pose2d pose, double magnitude, double direction) {
        double x = pose.getX() + magnitude * Math.cos(direction);
        double y = pose.getY() + magnitude * Math.sin(direction);
        return new Pose2d(x, y, Rotation2d.fromRadians(direction));
    }
}
