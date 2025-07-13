package com.team6560.frc2025.utility.Pathing;

import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** A class to simulate a curved path in 2D space, with params for max velocity and acceleration. 
*/
public class CubicPath extends Path{

    private double x3 = 0.0;
    private double x2 = 0.0;
    private double x1 = 0.0;
    private double x0 = 0.0;

    private double y3 = 0.0;
    private double y2 = 0.0;
    private double y1 = 0.0;
    private double y0 = 0.0;



    /** Defines a {@link CubicPath} in 2 dimensions. Translation is handled with a cubic bezier curve.
     * @param startPose The start pose
     * @param endPose The end pose
     * @param startControlHeading The control point for the start of the curve, which defines the initial heading.
     * @param endControlHeading The control point for the end of the curve, which defines the final heading.
     * @param maxVelocity Maximum velocity
     * @param maxAt Max tangential accel
     */
    public CubicPath(Pose2d startPose, Pose2d endPose, Pose2d startControlHeading, Pose2d endControlHeading, 
                        double maxVelocity, double maxAt, double staticCof) {
        super(startPose, endPose, startControlHeading, endControlHeading, maxVelocity, maxAt, staticCof);

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

    /** Calculates position of curve with time interval t 
     * @param t the time parameter, in the range [0, 1]
     * @return position of robot at time t
    */
    @Override
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
    @Override
    public Translation2d calculateFirstDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double dx = 3 * x3 * Math.pow(t, 2) + 2 * x2 * t + x1;
        double dy = 3 * y3 * Math.pow(t, 2) + 2 * y2 * t + y1;

        return new Translation2d(dx, dy);
    }

    @Override
    public Translation2d calculateSecondDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double ddx = 6 * x3 * t + 2 * x2;
        double ddy = 6 * y3 * t + 2 * y2;

        return new Translation2d(ddx, ddy);
    }
}
