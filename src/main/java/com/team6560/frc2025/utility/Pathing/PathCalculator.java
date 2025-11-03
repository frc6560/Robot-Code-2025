package com.team6560.frc2025.utility.Pathing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.team6560.frc2025.Constants;
import com.team6560.frc2025.utility.Setpoint;

// TODO LIST:
// equalize headings

/** A method to generate a smooth path to a pose of our choice. */
public class PathCalculator {
    public Setpoint startPose;
    public Setpoint endPose;

    public double startFinalControlLength;
    public double waypointInitialControlLength;
    public double waypointFinalControlLength;
    public double endInitialControlLength;

    public final double RADIUS = 1.8; // radius of the circle around the reef in meters

    public final Pose2d BLUE_REEF_CENTER = new Pose2d(4.48, 4, new Rotation2d(0));
    public final Pose2d RED_REEF_CENTER = new Pose2d(13.05, 4, new Rotation2d(0));

    public Pose2d reefCenter;

    public boolean hasObstacle; 

    /** Designed to return a path from our start to end pose, while avoiding any possible obstacles.
     * @param currentPose the current pose of the robot
     * @param finalPose the target pose
     */
    public PathCalculator(Setpoint currentPose, Setpoint finalPose){
        this.startPose = currentPose;
        this.endPose = finalPose;

        this.hasObstacle = getObstacleExists(currentPose.getSetpointPose(), finalPose.getSetpointPose());

        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red); 
        
        reefCenter = (alliance == Alliance.Blue) ? BLUE_REEF_CENTER : RED_REEF_CENTER;
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
    public boolean getObstacleExists(Pose2d firstPose, Pose2d secondPose){
        Pose2d reefCenter;
        if(DriverStation.getAlliance().get() == Alliance.Blue){
            reefCenter = BLUE_REEF_CENTER;
        }
        else {
            reefCenter = RED_REEF_CENTER;
        }
        Translation2d firstDisplacement = firstPose.getTranslation().minus(reefCenter.getTranslation());
        Translation2d secondDisplacement = secondPose.getTranslation().minus(reefCenter.getTranslation());
        double diff = Math.abs(Math.atan2(secondDisplacement.getY(), secondDisplacement.getX()) - 
                        Math.atan2(firstDisplacement.getY(), firstDisplacement.getX()));
        return diff > 120.0;
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

    
    public boolean hasObstacle() {
        return hasObstacle;
    }
    

    // From here on out, denote our start point as A, our end point as B. Define omega as the circle circumscribing the reef.


    /** Gets the two intersections points of AB with the circle circumscribed around the reef
     * @return An array of two Translation2d objects representing the intersection of AB with omega.
    */
    public Translation2d[] getCircleIntersections(){

        // models the line AB as a vector from A to B. Form is r(t) = (px, py) + t(dx, dy), where t is a time parameter.
        double px = startPose.getSetpointPose().getX();
        double py = startPose.getSetpointPose().getY();
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
            return new Translation2d[] {}; // No intersection points
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

    /** A simple straight line distance based solution for getting path heading lengths*/
    public double calculateControlLengths(Pose2d firstPose, Pose2d secondPose){
        Translation2d displacement = secondPose.getTranslation().minus(firstPose.getTranslation());
        double distance = displacement.getNorm();
        return MathUtil.clamp(distance / 3.0, 0.5, 3.0);
    }


    /** This is for the special case in which we need to generate a path around an obstacle. 
     * @return a PathGroup object that contains two paths: one to a control point generated by going 1.5x robot length around the obstacle, and one to the target pose.
    */
    public PathGroup calculatePathGroup(){
        if (!hasObstacle) return null;

        Pose2d circleMidpoint = getMidpoint(getCircleIntersections()[0], getCircleIntersections()[1]);
        getNormalVector(getCircleIntersections()[0], getCircleIntersections()[1]);

        // Calculates the middle control point. Because the normal vector may have been inverted, we do this the long way.
        Translation2d disp = circleMidpoint.getTranslation().minus(reefCenter.getTranslation());
        double angle = Math.atan2(disp.getY(), disp.getX());
        double clearance = 1.5 * Math.hypot(Constants.ROBOT_LENGTH, Constants.ROBOT_WIDTH);
        double distanceToCircle = Math.max(this.RADIUS - circleMidpoint.getTranslation().minus(reefCenter.getTranslation()).getNorm(), 0);
        double magnitude = distanceToCircle + clearance;

        Pose2d waypointWithoutRotation = getPoseDirectionFrom(circleMidpoint, magnitude, angle);
        Pose2d waypointPose = new Pose2d(waypointWithoutRotation.getTranslation(), new Rotation2d(calculateControlAngle()));

        Setpoint waypoint = new Setpoint(
            waypointPose.getX(),
            waypointPose.getY(),
            waypointPose.getRotation().getRadians(),
            0, // vx
            0, // vy
            0  // omega
        );

        // Calculates the control lengths for our paths.
        this.startFinalControlLength = calculateControlLengths(startPose.getSetpointPose(), waypointPose);
        this.waypointInitialControlLength = calculateControlLengths(startPose.getSetpointPose(), waypointPose);

        this.waypointFinalControlLength = calculateControlLengths(waypointPose, endPose.getSetpointPose());
        this.endInitialControlLength = calculateControlLengths(waypointPose, endPose.getSetpointPose());

        // Finally calculates our paths.
        Path firstPath = new Path(
            this.startPose,
            waypoint,
            new Pose2d(), // FIX LATER!
            getControlPoints(waypointPose)[0],
            5.0, // tune
            4.0, // tune
            3.14,
            6.28);

        Path secondPath = new Path(
            waypoint,
            this.endPose,
            getControlPoints(waypointPose)[1],
            new Pose2d(),
            5.0,
            4.0,
            3.14,
            6.28);

        return new PathGroup(firstPath, secondPath, 5.0, 4.0, 3.14, 6.28);
    }
}
