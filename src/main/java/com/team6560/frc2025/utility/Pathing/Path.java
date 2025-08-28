package com.team6560.frc2025.utility.Pathing;

import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


/** A cubic Bezier curve path object.
 * @author fwen2026 */
public class Path {
    private final Setpoint startPose;
    private final Setpoint endPose;

    private final Pose2d startControlHeading;
    private final Pose2d endControlHeading;

    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile.State endState;

    private final TrapezoidProfile.State startRotation;
    private TrapezoidProfile.State currentRotation;
    private final TrapezoidProfile.State endRotation;

    // Profiles handling translation and rotation
    private final TrapezoidProfile translationProfile;
    private final TrapezoidProfile rotationProfile;

    private final int LOOKUP_RES = 1000;
    private double[] arcLengthChart = new double[LOOKUP_RES + 1];

    private double x3 = 0.0;
    private double x2 = 0.0;
    private double x1 = 0.0;
    private double x0 = 0.0;

    private double y3 = 0.0;
    private double y2 = 0.0;
    private double y1 = 0.0;
    private double y0 = 0.0;


    /** Defines a {@link Path} in 2 dimensions. Translation is handled via a Bézier curve and trapezoidal profile. 
     * Rotation is handled using a separate profile controlled by maxOmega and maxAlpha.
     * 
     * @param startPose The start pose
     * @param endPose The end pose
     * @param startControlHeading The control point for the start of the curve, which defines the initial heading. Quintic bezier curves have an additional two control points.
     * @param endControlHeading The control point for the end of the curve, which defines the final heading. 
     * @param maxVelocity Maximum velocity
     * @param maxAt Max tangential acceleration allowed on the path
     * @param maxOmega Maximum angular velocity, in radians/s
     * @param maxAlpha Maximum angular acceleration, in radians/s^2
     * 
     */
    public Path(Setpoint startPose, Setpoint endPose, Pose2d startControlHeading, Pose2d endControlHeading, 
                        double maxVelocity, double maxAt, double maxOmega, double maxAlpha) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.startControlHeading = startControlHeading;
        this.endControlHeading = endControlHeading;

        // Actually defines our curve
        // defines x component for the cubic Bézier curve
        this.x3 = -startPose.x + 3 * startControlHeading.getX() - 3 * endControlHeading.getX() + endPose.x;
        this.x2 = 3 * startPose.x - 6 * startControlHeading.getX() + 3 * endControlHeading.getX();
        this.x1 = -3 * startPose.x + 3 * startControlHeading.getX();
        this.x0 = startPose.x;

        // defines y components as well.
        this.y3 = -startPose.y + 3 * startControlHeading.getY() - 3 * endControlHeading.getY() + endPose.y;
        this.y2 = 3 * startPose.y - 6 * startControlHeading.getY() + 3 * endControlHeading.getY();
        this.y1 = -3 * startPose.y + 3 * startControlHeading.getY();
        this.y0 = startPose.y;

        // generate a lookup table for arc length to time
        generateLookupTable();

        // Sets up the trapezoidal profile start and end states... as well as the actual profiles
        // translation
        this.currentState = new TrapezoidProfile.State(0, startPose.getSpeed());
        this.endState = new TrapezoidProfile.State(getArcLength(), endPose.getSpeed());

        this.translationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAt));

        // rotation
        this.startRotation = new TrapezoidProfile.State(startPose.theta, startPose.omega);
        this.currentRotation = new TrapezoidProfile.State(startPose.theta, startPose.omega);
        this.endRotation = new TrapezoidProfile.State(endPose.theta, endPose.omega);
        this.rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxOmega, maxAlpha));
    }

    /** Getter methods*/

    /** Gets start pose */
    public Pose2d getStartPose() {
        return startPose.getSetpointPose();
    }

    /** Gets end pose */
    public Pose2d getEndPose() {
        return endPose.getSetpointPose();
    }

    /** Gets the start pose velocity */
    public double getStartVelocity() {
        return startPose.getSpeed();
    }

    /** Gets the end pose velocity */
    public double getEndVelocity(){
        return endPose.getSpeed();
    }

    /** Gets the start pose heading */
    public Pose2d getStartControlHeading() {
        return startControlHeading;
    }

    /** Gets the end pose heading */
    public Pose2d getEndControlHeading() {
        return endControlHeading;
    }


    /** This is a helper method to obtain the net arc length of the curve. Uses a discrete approximation of numerical integration. See below for more information!
     * @return The arc length of the path
     */
    public double getArcLength(){
        return arcLengthChart[LOOKUP_RES];
    }


    /** Calculates position of curve with time interval t 
     * @param t the time parameter, in the range [0, 1]
     * @return position of robot at time t
    */
    public Translation2d calculatePosition(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double x = x3 * Math.pow(t, 3) + x2 * Math.pow(t, 2) + x1 * t + x0;
        double y = y3 * Math.pow(t, 3) + y2 * Math.pow(t, 2) + y1 * t + y0;

        return new Translation2d(x, y);
    }

    /** Calculates the first derivative at point t
     * @param t the time parameter, in the range [0, 1]
     */
    public Translation2d calculateFirstDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double dx = 3 * x3 * Math.pow(t, 2) + 2 * x2 * t + x1;
        double dy = 3 * y3 * Math.pow(t, 2) + 2 * y2 * t + y1;

        return new Translation2d(dx, dy);
    }

    /** Calculates the second derivative at point t
     * @param t the time parameter, in the range [0, 1]
     */
    public Translation2d calculateSecondDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double ddx = 6 * x3 * t + 2 * x2;
        double ddy = 6 * y3 * t + 2 * y2;

        return new Translation2d(ddx, ddy);
    }


    /** This generates a lookup table to obtain different arc lengths.
     * @return An array of arc lengths for the path at different time intervals, with resolution 1/1000 of the total time.
     */
    public void generateLookupTable(){
        arcLengthChart[0] = 0.0; // start here
        Translation2d p0 = calculatePosition(0.0);
        for(int i = 1; i <= LOOKUP_RES; i++){
            double t = (double)i / LOOKUP_RES; // time parameter
            Translation2d p1 = calculatePosition(t);
            arcLengthChart[i] = arcLengthChart[i - 1] + p0.getDistance(p1);
            p0 = p1;
        }
    }


    /** This gives a decent approximation of the best time value for a certain arc length. 
     * @param arcLength The arc length starting from start pose
     * @return A time value corresponding to our length
    */
    public double getTimeForArcLength(double arcLength){
        if(arcLength < 0 || arcLength > getArcLength()){
            throw new IllegalArgumentException("you're chopped. (arc length in 0, total arc length)");
        }
        int low = 0;
        int high = LOOKUP_RES;

        // Binary searches bc i was lazy
        while (low <= high){
            int mid = (low + high) / 2;
            if (arcLengthChart[mid] < arcLength){
                low = mid + 1;
            } else{
                high = mid - 1;
            }
        }
        return (double)low / LOOKUP_RES;
    }


    /** Calculates the curvature of our path at time t, for better velocity control when turning. 
     * @param t the time parameter
     * @return The curvature of the path at time t.
     */
    public double getCurvature(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        Translation2d firstDerivative = calculateFirstDerivative(t);
        Translation2d secondDerivative = calculateSecondDerivative(t);
        return (firstDerivative.getX() * secondDerivative.getY() - firstDerivative.getY() * secondDerivative.getX()) 
                / Math.pow(firstDerivative.getNorm(), 3);
    }


    /** This is another helper method to compute the normalized velocity vector T(t). 
     * @param t the time parameter
     * @return A normalized vector representing the direction of the path at time t.
     */
    public Translation2d getNormalizedVelocityVector(double t) {
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        Translation2d tangentVector = calculateFirstDerivative(t);
        return tangentVector.div(tangentVector.getNorm() + 0.001); 
    }

    
    /** Calculates the next position of the path for the robot to target. Returns as a Setpoint object. 
     * The reason we need the current rotation is because mod 360 shenanigans 
     * @param t the time parameter, currentRotation the current rotation of the robot
     * @param currentRotation the current rotation of the robot
     */
    public Setpoint calculate(double rotation){
        // Translation
        TrapezoidProfile.State translationalSetpoint = translationProfile.calculate(0.02, currentState, endState);
        double timeParam = getTimeForArcLength(translationalSetpoint.position);
        Translation2d translationalTarget = calculatePosition(timeParam);
        currentState.position = translationalSetpoint.position;
        currentState.velocity = translationalSetpoint.velocity;

        // Rotation
        double rotationalPose = rotation;
        double errorToGoal = MathUtil.angleModulus(endRotation.position - rotationalPose);
        double errorToSetpoint = MathUtil.angleModulus(startRotation.position - rotationalPose);

        // gets rid of mod 2pi issues
        endRotation.position = rotationalPose + errorToGoal;
        currentRotation.position = rotationalPose + errorToSetpoint;

        // finally computes next rotation state 
        State rotationalSetpoint = rotationProfile.calculate(0.02, currentRotation, endRotation);
        currentRotation.position = rotationalSetpoint.position;
        currentRotation.velocity = rotationalSetpoint.velocity;

        return new Setpoint(translationalTarget.getX(), 
                            translationalTarget.getY(), 
                            rotationalSetpoint.position, 
                            getNormalizedVelocityVector(timeParam).getX() * translationalSetpoint.velocity, 
                            getNormalizedVelocityVector(timeParam).getY() * translationalSetpoint.velocity,
                            rotationalSetpoint.velocity);
    }
}
