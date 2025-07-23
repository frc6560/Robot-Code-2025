package com.team6560.frc2025.utility.Pathing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.team6560.frc2025.Constants;

// TODO LIST:
// write generation algorithm for a waypoint. (done)
// for obstacle get the segment intersecting circle, find perpendicular bisector, and go a tunable distance! (done)
// generate path
// equalize headings ()
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

    public final double RADIUS = 1.8; // radius of the circle around the reef in meters

    public final Pose2d BLUE_REEF_CENTER = new Pose2d(4.48, 4, new Rotation2d(0));
    public final Pose2d RED_REEF_CENTER = new Pose2d(13.05, 4, new Rotation2d(0));

    public Pose2d reefCenter;

    public boolean useQuintic; // LMFAO this needs to be renamed

    /** Designed to return a path from our start to end pose, while avoiding any possible obstacles.
     * @param currentPose the current pose of the robot
     * @param finalPose the target pose
     */
    public PathCalculator(Pose2d currentPose, Pose2d finalPose){
        this.startPose = currentPose;
        this.endPose = finalPose;

        if(Math.abs(getRegion(currentPose) - getRegion(finalPose)) > 1){
            useQuintic = true;
        } else {
            useQuintic = false;
        }
        // This is fine because we initialize path calculator objects following robotinit
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            reefCenter = BLUE_REEF_CENTER;
        } else{
            reefCenter = RED_REEF_CENTER;
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


    /** Gets the "region" of a specific Pose2D according to its theta value. 
     * @return The region of a specific Pose2D.
    */
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


    /** Gets the two middle control points for our bezier spline. The first control point is our initial control point. The second is our final.
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
            DriverStation.reportError("You cannot generate a quintic path right now!", false);
            return null;
        }
        // Calculates the angle bisector between our two poses. 
        double angle = startPose.getRotation().getRadians() 
                        + MathUtil.angleModulus(endPose.getRotation().getRadians() - startPose.getRotation().getRadians()) / 2.0;
        
        // Calculates the control length based on the angle difference and the displacement.
        double controLengthConstant = 1.0; // subject to tuning
        Translation2d displacement = endPose.getTranslation().minus(startPose.getTranslation());

        double angleDifference = Math.abs(angle, Math.atan2(displacement.getY(), displacement.getX()));
        this.startFinalControlLength = displacement.getNorm() * Math.abs(Math.sin(angleDifference)) * controLengthConstant;
        this.endInitialControlLength = startFinalControlLength; // symmetry

        // Gets the control points for the start and end poses.
        Pose2d startControlHeading = getPoseDirectionFrom(startPose, 
                                                            startFinalControlLength, 
                                                            angle);
        Pose2d endControlHeading = getPoseDirectionFrom(endPose, 
                                                            endInitialControlLength, 
                                                            angle + Math.PI);

        return new Path(startPose,
                        endPose, 
                        startControlHeading, 
                        endControlHeading, 
                        3.0, 3.0,
                        Constants.WHEEL_COF);
    }


    // From here on out, denote our start point as A, our end point as B. Define omega as the circle circumscribing the reef.


    /** Gets the two intersections points of AB with the circle circumscribed around the reef
     * @return An array of two Translation2d objects representing the intersection of AB with omega.
    */
    public Translation2d[] getCircleIntersections(){

        // models the line AB as a vector from A to B. Form is r(t) = (px, py) + t(dx, dy), where t is a time parameter.
        double px = startPose.getX();
        double py = startPose.getY();
        Translation2d pathDisplacement = endPose.getTranslation().minus(startPose.getTranslation());
        double dx = pathDisplacement.getX();
        double dy = pathDisplacement.getY();

        // models our circle.
        double cx = reefCenter.getX();
        double cy = reefCenter.getY();
        double rSquared = Math.pow(RADIUS, 2);

        // the intersection points can be found by substituting the parametrization into the circle equation and solving for t, then refactoring into r.
        double A = Math.pow(dx, 2) + Math.pow(dy, 2);
        double B = 2 * (dx * (px - cx) + dy * (py - cy));
        double C = Math.pow(px - cx, 2) + Math.pow(py - cy, 2) - rSquared;

        // Checks for discriminant > 0
        double delta = Math.pow(B, 2) - 4 * A * C;
        if (delta < 0) {
            throw new IllegalArgumentException("you're chopped. (line does not intersect circle)");
        }

        // bash :D
        double firstTValue = (-B + Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);
        double secondTValue = (-B - Math.sqrt(Math.pow(B, 2) - 4 * A * C)) / (2 * A);

        Translation2d firstIntersection = new Translation2d(
            px + firstTValue * dx, 
            py + firstTValue * dy
        );
        Translation2d secondIntersection = new Translation2d(
            px + secondTValue * dx, 
            py + secondTValue * dy
        ); 

        return new Translation2d[] {firstIntersection, secondIntersection};
    }


    /** Gets the midpoint of two points (rotation doesn't even matter) */
    public Pose2d getMidpoint(Translation2d a, Translation2d b) {
        double x = (a.getX() + b.getX()) / 2.0;
        double y = (a.getY() + b.getY()) / 2.0;
        return new Pose2d(x, y, Rotation2d.fromDegrees(0));
    }


    /** Gets the normal and normalized vector to AB.*/
    public Translation2d getNormalVector(Translation2d a, Translation2d b) {
        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();
        Translation2d normal = new Translation2d(-dy, dx); 
        return normal.div(normal.getNorm() + 1E-6); // Normal vector is perpendicular to AB
    }


    /** Calculates a control angle for our middle waypoint*/
    public double calculateControlAngle(){
        Translation2d displacement = endPose.getTranslation().minus(startPose.getTranslation());
        return Math.atan2(displacement.getY(), displacement.getX());
    }


    /** This is for the special case in which we need to generate a path around an obstacle. 
     * @return a PathGroup object that contains two paths: one to a control point generated by going 1.5x robot length around the obstacle, and one to the target pose.
    */
    public PathGroup calculatePathGroup(){
        if(!useQuintic){
            DriverStation.reportError("You cannot generate a PathGroup right now!", false);
            return null;
        }
        Pose2d circleMidpoint = getMidpoint(getCircleIntersections()[0], getCircleIntersections()[1]);
        Translation2d normalVector = getNormalVector(getCircleIntersections()[0], getCircleIntersections()[1]);

        // Calculates the middle control point. Because the normal vector may have been inverted, we do this the long way.
        Translation2d disp = circleMidpoint.getTranslation().minus(reefCenter.getTranslation());
        double angle = Math.atan2(disp.getY(), disp.getX());
        double magnitude = (this.RADIUS - circleMidpoint.getTranslation().minus(reefCenter.getTranslation()).getNorm()) 
                            + 1.5 * Math.sqrt(Math.pow(Constants.ROBOT_LENGTH, 2) + Math.pow(Constants.ROBOT_WIDTH, 2)); 
        Pose2d waypointWithoutRotation = getPoseDirectionFrom(circleMidpoint, magnitude, angle);

        Pose2d waypoint = new Pose2d(waypointWithoutRotation.getTranslation(), new Rotation2d(calculateControlAngle()));

        // Calculates each control point length. Path 1 and Path 2 are handled separately, unlike our quintic curve.

        // Finally calculates our paths.
        Path firstPath = new Path(
            this.startPose,
            waypoint,
            new Pose2d(), // FIX LATER!
            getControlPoints(waypoint)[0],
            0.5, // tune
            0.5,
            Constants.WHEEL_COF);

        Path secondPath = new Path(
            waypoint,
            this.endPose,
            getControlPoints(waypoint)[1],
            new Pose2d(),
            0.5,
            0.5,
            Constants.WHEEL_COF);

        return new PathGroup(firstPath, secondPath, 0.5, 0.5, Constants.WHEEL_COF);
    }
}
