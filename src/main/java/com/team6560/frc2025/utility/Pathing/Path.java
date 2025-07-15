package com.team6560.frc2025.utility.Pathing;

import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Path {
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
    private double AT;
    private final double MAX_A;
    private double MAX_AC;

    /** Defines a {@link Path} in 2 dimensions. Translation is handled via a Bézier curve and trapezoidal profile. Rotation is handled linearly.
     * @param startPose The start pose
     * @param endPose The end pose
     * @param startControlHeading The control point for the start of the curve, which defines the initial heading. Quintic bezier curves have an additional two control points.
     * @param endControlHeading The control point for the end of the curve, which defines the final heading. 
     * @param maxVelocity Maximum velocity
     * @param maxAt Max tangential accel
     */
    public Path(Pose2d startPose, Pose2d endPose, Pose2d startControlHeading, Pose2d endControlHeading, 
                        double maxVelocity, double maxAt, double staticCof) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.startControlHeading = startControlHeading;
        this.endControlHeading = endControlHeading;

        // sets up the trapezoidal profile start and end states...
        this.startState = new TrapezoidProfile.State(0, 0);
        this.endState = new TrapezoidProfile.State(getArcLength(), 0);

        // ...and the profiles themselves
        this.MAX_VELOCITY = maxVelocity;


        this.translationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAt));

        // finally generates a lookup table for reference.
        generateLookupTable();

        // and calculates the maximum acceleration values.
        MAX_A = staticCof * 9.81;
        AT = 0;
        MAX_AC = Math.sqrt(Math.pow(MAX_A, 2) - Math.pow(AT, 2)); // centripetal acceleration
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


    /** This is a helper method to obtain the net arc length of the curve. Uses a discrete approximation of numerical integration. See below for more information!
     * @return The arc length of the path
     */
    public double getArcLength(){
        return arcLengthChart[LOOKUP_RES];
    }


    /** Stubs to be overridden. */
    public Translation2d calculatePosition(double t){
        return new Translation2d();
    }

    public Translation2d calculateFirstDerivative(double t){
        return new Translation2d();
    }

    public Translation2d calculateSecondDerivative(double t){
        return new Translation2d();
    }

    /** This calculates the maximum possible centripetal acceleration at any given point. */
    public void calculateMaxAc(){
        if(AT > MAX_A)  {
            throw new IllegalArgumentException("you're chopped (max tangential acceleration cannot be greater than max acceleration).");
        }
        MAX_AC = Math.sqrt(Math.pow(MAX_A, 2) - Math.pow(AT, 2)); // centripetal acceleration
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
     * @param t the time parameter, currentRotation the current rotation of the robot
     * @param currentRotation the current rotation of the robot
     */
    public Setpoint calculate(double t, double currentRotation){
        // Translation
        TrapezoidProfile.State translationalSetpoint = translationProfile.calculate(t, startState, endState);
        TrapezoidProfile.State previousState = translationProfile.calculate(t - 0.02, startState, endState);

        double timeParam = getTimeForArcLength(translationalSetpoint.position);
        Translation2d translationalTarget = calculatePosition(timeParam);

        // Calculates max ac based upon at and takes the maximum velocity possible.
        AT = (translationalSetpoint.velocity - previousState.velocity) / 0.02; // change in velocity over time
        AT = MathUtil.clamp(AT, -MAX_A, MAX_A); // clamps to max acceleration
        calculateMaxAc();
        double velocityMultiplier = Math.min(translationalSetpoint.velocity, Math.sqrt(MAX_AC / getCurvature(timeParam) + 1E-6));

        // Rotation
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
