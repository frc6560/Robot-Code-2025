package com.team6560.frc2025.utility.Pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

// TODO LIST:
// no hardbaking
// write generation algorithm for a waypoint.
// use our regions 1-6 method still. cubic for no obstacle quintic for obstacle.
// for no obstacle angle bisector works fine
// for obstacle get the segment intersecting circle, find perpendicular bisector, and go a tunable distance!
// generate path
// equalize headings
// go!

/** A method to generate a smooth path to a pose of our choice. */
public class PathCalculator {
    public Path path;

    public Pose2d startPose;
    public Pose2d endPose;

    public double startFinalControlLength;
    public double waypointFinalControlLength;
    public double waypointInitialControlLength;
    public double endInitialControlLength;

    public final Pose2d BLUE_REEF_CENTER = new Pose2d(4.48, 4, new Rotation2d(0));
    public final Pose2d RED_REEF_CENTER = new Pose2d(13.05, 4, new Rotation2d(0));

    public boolean useQuintic;

    public PathCalculator(Pose2d currentPose, Pose2d finalPose){
        this.startPose = currentPose;
        this.endPose = finalPose;

        if(Math.abs(getRegion(currentPose) - getRegion(finalPose)) > 1){
            useQuintic = true;
        } else {
            useQuintic = false;
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


    /** Gets the "region" of a specific Pose2D according to its theta value. */
    public int getRegion(Pose2d pose){
        Pose2d reefCenter;
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            reefCenter = BLUE_REEF_CENTER;
        }
        else {
            reefCenter = RED_REEF_CENTER;
        }
        Translation2d displacement = pose.getTranslation().minus(reefCenter.getTranslation());
        double poseAngle = Rotation2d.fromRadians(Math.atan2(displacement.getY(), displacement.getX())).getDegrees();

        return (int) (poseAngle + 30.0) / 60; // 0-5, inclusive
    }


    /** Gets the two middle control points for a quintic bezier curve. The first control point is our initial control point. The second is our final.
     * @param waypoint The waypoint to get the control points from
    */
    public Pose2d[] getControlPoints(Pose2d waypoint) {
        Pose2d firstControlHeading = getPoseDirectionFrom(waypoint, waypointInitialControlLength, 
                waypoint.getRotation().getRadians() + Math.PI);
        Pose2d secondControlHeading = getPoseDirectionFrom(waypoint, waypointFinalControlLength, 
                waypoint.getRotation().getRadians());
        return new Pose2d[] {firstControlHeading, secondControlHeading};
    }

    
    public boolean getUseQuintic() {
        return useQuintic;
    }

    /** Obtains a cubic path from the start pose to the end pose, with a waypoint in between. Only use if {@link #getUseQuintic()} is false.
     * @param waypoint The waypoint to use
     * @return A path from the start pose to the end pose, with a waypoint in between
     */
    public Path generateCubicPath(){
        if(useQuintic){
            DriverStation.reportError("You cannot generate a cubic path with a quintic path calculator!", false);
            return null;
        }
        Pose2d startControlHeading = getPoseDirectionFrom(startPose, startFinalControlLength, 
                startPose.getRotation().getRadians() + Math.PI);
        Pose2d endControlHeading = getPoseDirectionFrom(endPose, endInitialControlLength, 
                endPose.getRotation().getRadians());

        return new CubicPath(startPose, endPose, startControlHeading, endControlHeading, 
                path.getMaxVelocity(), path.getMaxAt(), path.getStaticCof());
    }
}
