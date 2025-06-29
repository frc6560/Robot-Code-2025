package com.team6560.frc2025.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A class to simulate a curved path in 2D space, with params for max velocity and acceleration. 
*/
public class CurvedPath {
    // These should NOT be modified.
    private final Pose2d startPose;
    private final Pose2d endPose;

    private final Pose2d startControlHeading;
    private final Pose2d endControlHeading;

    private TrapezoidProfile.State startState;
    private final TrapezoidProfile.State endState;

    private final TrapezoidProfile translationProfile;

    private final int LOOKUP_RES = 1000;
    private double[] arcLengthChart = new double[LOOKUP_RES + 1];

    private final double MAX_VELOCITY;
    private final double MAX_AT;
    private final double MAX_AC;

    private double x3 = 0.0;
    private double x2 = 0.0;
    private double x1 = 0.0;
    private double x0 = 0.0;

    private double y3 = 0.0;
    private double y2 = 0.0;
    private double y1 = 0.0;
    private double y0 = 0.0;

    // preset two positions around the reef. first heading from current position is standard (1 or something). second one is also standard, but is pre-baked.
    // you can do this approach with pretty much any game


    /** Defines a {@link CurvedPath} in 2 dimensions. Translation is handled via a Bézier curve and trapezoidal profile. Rotation is handled linearly.
     * @param startPose The start pose
     * @param endPose The end pose
     * @param startControlHeading The control point for the start of the curve, which defines the initial heading.
     * @param endControlHeading The control point for the end of the curve, which defines the final heading.
     * @param maxVelocity Maximum velocity
     * @param maxAt Max tangential accel
     * @param maxAc Max centripetal accel
     */
    public CurvedPath(Pose2d startPose, Pose2d endPose, Pose2d startControlHeading, Pose2d endControlHeading, 
                        double maxVelocity, double maxAt, double maxAc) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.startControlHeading = startControlHeading;
        this.endControlHeading = endControlHeading;
        
        generateBezierCurve(startPose, endPose, startControlHeading, endControlHeading);

        // sets up the trapezoidal profile start and end states...
        this.startState = new TrapezoidProfile.State(0, 0);
        this.endState = new TrapezoidProfile.State(getArcLength(), 0);

        // ...and the profiles themselves
        this.MAX_VELOCITY = maxVelocity;
        this.MAX_AC = maxAc;
        this.MAX_AT = maxAt;

        this.translationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, MAX_AT));

        // finally generates a lookup table for reference.
        generateLookupTable();
    }

    /** An alternative initialization with something similar to a Hermite spline 
     * @param startHeading Start heading, vector from start pose
     * @param endHeading End heading, vector from end pose.
    */
    public CurvedPath(Pose2d startPose, Pose2d endPose, Translation2d startHeading, Translation2d endHeading,
                        double maxVelocity, double maxAt, double maxAc){
            Pose2d startControlHeading = new Pose2d(startPose.getTranslation().plus(startHeading), new Rotation2d(0));
            Pose2d endControlHeading = new Pose2d(endPose.getTranslation().plus(endHeading), new Rotation2d(0));

            generateBezierCurve(startPose, endPose, startControlHeading, endControlHeading);

            this.startPose = startPose;
        this.endPose = endPose;
        this.startControlHeading = startControlHeading;
        this.endControlHeading = endControlHeading;
        
        generateBezierCurve(startPose, endPose, startControlHeading, endControlHeading);

        // sets up the trapezoidal profile start and end states...
        this.startState = new TrapezoidProfile.State(0, 0);
        this.endState = new TrapezoidProfile.State(getArcLength(), 0);

        // ...and the profiles themselves
        this.MAX_VELOCITY = maxVelocity;
        this.MAX_AC = maxAc;
        this.MAX_AT = maxAt;

        this.translationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, MAX_AT));

        // finally generates a lookup table for reference.
        generateLookupTable();
    }

    /** A method for generating a Bezier curve
     * @param
     */
    public void generateBezierCurve(Pose2d startPose, Pose2d endPose, Pose2d startControlHeading, Pose2d endControlHeading){
        // defines x component for the cubic Bézier curve
        this.x3 = -startPose.getX() + 3 * startControlHeading.getX() - 3 * endControlHeading.getX() + endPose.getX();
        this.x2 = 3 * startPose.getX() - 6 * startControlHeading.getX() + 3 * endControlHeading.getX();
        this.x1 = -3 * startPose.getX() + 3 * startControlHeading.getX();
        this.x0 = startPose.getX();
 
        // defines y components as well.
        this.y3 = -startPose.getY() + 3 * startControlHeading.getY() - 3 * endControlHeading.getY() + endPose.getY();
        this.y2 = 3 * startPose.getY() - 6 * startControlHeading.getY() + 3 * endControlHeading.getY();
        this.y1 = -3 * startPose.getY() + 3 * startControlHeading.getY();
        this.y0 = startPose.getY();
    }

    /** Getter methods*/

    /** Gets start pose */
    public Pose2d getStartPose() {
        return startPose;
    }

    /** Gets end pose */
    public Pose2d getEndPose() {
        return endPose;
    }

    /** Gets the start pose heading */
    public Pose2d getStartControlHeading() {
        return startControlHeading;
    }

    /** Gets the end pose heading */
    public Pose2d getEndControlHeading() {
        return endControlHeading;
    }

    /** Calculation stuff */

    /** calculates position of curve with time interval t */
    public Translation2d calculatePosition(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double x = x3 * Math.pow(t, 3) + x2 * Math.pow(t, 2) + x1 * t + x0;
        double y = y3 * Math.pow(t, 3) + y2 * Math.pow(t, 2) + y1 * t + y0;

        return new Translation2d(x, y);
    }


    /** This is a helper method to obtain the net arc length of the curve. Uses a discrete approximation of numerical integration. See below for more information!
     * @return The arc length of the path
     */
    public double getArcLength(){
        return arcLengthChart[LOOKUP_RES];
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

    /** Calculates the curvature of our path at time t, for better velocity control when turning. 
     * @param t the time parameter
     * @return The curvature of the path at time t.
     */
    public double getCurvature(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        Translation2d firstDerivative = new Translation2d(3 * x3 * Math.pow(t, 2) + 2 * x2 * t + x1, 
                                                        3 * y3 * Math.pow(t, 2) + 2 * y2 * t + y1);
        Translation2d secondDerivative = new Translation2d(6 * x3 * t + 2 * x2, 
                                                        6 * y3 * t + 2 * y2);
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
        Translation2d tangentVector = new Translation2d(3 * x3 * Math.pow(t, 2) + 2 * x2 * t + x1, 
                                                        3 * y3 * Math.pow(t, 2) + 2 * y2 * t + y1);
        return tangentVector.div(tangentVector.getNorm() + 0.001); 
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


    /** Calculates the next position of the path for the robot to target. Returns as a Setpoint object.
     * @params 
     */
    public Setpoint calculate(double t, double currentRotation){
        // Translation
        TrapezoidProfile.State translationalSetpoint = translationProfile.calculate(t, startState, endState);

        double timeParam = getTimeForArcLength(translationalSetpoint.position);
        Translation2d translationalTarget = calculatePosition(timeParam);

        double velocityMultiplier = Math.min(translationalSetpoint.velocity, Math.sqrt(MAX_AC / getCurvature(t) + 1E-6));

        double thetaTarget = MathUtil.interpolate(currentRotation, 
                                                endPose.getRotation().getRadians(), timeParam);

        return new Setpoint(translationalTarget.getX(), 
                            translationalTarget.getY(), 
                            thetaTarget, 
                            getNormalizedVelocityVector(timeParam).getX() * velocityMultiplier, 
                            getNormalizedVelocityVector(timeParam).getY() * velocityMultiplier,
                            0);
    }
}
