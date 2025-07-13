package com.team6560.frc2025.utility.Pathing;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class QuinticPath extends Path{
    private double x5 = 0.0;
    private double x4 = 0.0;
    private double x3 = 0.0;
    private double x2 = 0.0;
    private double x1 = 0.0;
    private double x0 = 0.0;

    private double y5 = 0.0;
    private double y4 = 0.0;
    private double y3 = 0.0;
    private double y2 = 0.0;
    private double y1 = 0.0;
    private double y0 = 0.0;

    public QuinticPath(Pose2d startPose, Pose2d endPose, Pose2d startControlHeading, Pose2d endControlHeading, 
                        double maxVelocity, double maxAt, double staticCof, Pose2d secondControlHeading, Pose2d thirdControlHeading) {
        super(startPose, endPose, startControlHeading, endControlHeading, maxVelocity, maxAt, staticCof);

        // defines x component for the quintic Bézier curve
        this.x5 = -startPose.getX() + 5 * startControlHeading.getX() - 10 * secondControlHeading.getX() + 10 * endControlHeading.getX() - endPose.getX();
        this.x4 = 5 * startPose.getX() - 20 * startControlHeading.getX() + 20 * secondControlHeading.getX() - 5 * endControlHeading.getX();
        this.x3 = -10 * startPose.getX() + 30 * startControlHeading.getX() - 30 * secondControlHeading.getX() + 10 * endControlHeading.getX();
        this.x2 = 10 * startPose.getX() - 15 * startControlHeading.getX() + 5 * secondControlHeading.getX();
        this.x1 = -5 * startPose.getX() + 5 * startControlHeading.getX();
        this.x0 = startPose.getX();

        // defines y components as well.
        this.y5 = -startPose.getY() + 5 * startControlHeading.getY() - 10 * secondControlHeading.getY() + 10 * endControlHeading.getY() - endPose.getY();
        this.y4 = 5 * startPose.getY() - 20 * startControlHeading.getY() + 20 * secondControlHeading.getY() - 5 * endControlHeading.getY();
        this.y3 = -10 * startPose.getY() + 30 * startControlHeading.getY() - 30 * secondControlHeading.getY() + 10 * endControlHeading.getY();
        this.y2 = 10 * startPose.getY() - 15 * startControlHeading.getY() + 5 * secondControlHeading.getY();
        this.y1 = -5 * startPose.getY() + 5 * startControlHeading.getY();
        this.y0 = startPose.getY();
    }

    /** Calculates position of curve with time interval t 
     * @param t the time parameter, in the range [0, 1]
     * @return position of robot at time t
     */
    @Override
    public Translation2d calculatePosition(double t) {
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException("t must be in the range [0, 1]");
        }

        double x = x5 * Math.pow(t, 5) + x4 * Math.pow(t, 4) + x3 * Math.pow(t, 3) +
                   x2 * Math.pow(t, 2) + x1 * t + x0;

        double y = y5 * Math.pow(t, 5) + y4 * Math.pow(t, 4) + y3 * Math.pow(t, 3) +
                   y2 * Math.pow(t, 2) + y1 * t + y0;

        return new Translation2d(x, y);
    }

    /** Calculates the first derivative at point t
     * @param t the time parameter, in the range [0, 1]
     */
    @Override
    public Translation2d calculateFirstDerivative(double t) {
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException("t must be in the range [0, 1]");
        }

        double dx = 5 * x5 * Math.pow(t, 4) + 4 * x4 * Math.pow(t, 3) + 3 * x3 * Math.pow(t, 2) +
                    2 * x2 * t + x1;

        double dy = 5 * y5 * Math.pow(t, 4) + 4 * y4 * Math.pow(t, 3) + 3 * y3 * Math.pow(t, 2) +
                    2 * y2 * t + y1;

        return new Translation2d(dx, dy);
    }

    @Override
    public Translation2d calculateSecondDerivative(double t) {
        if (t < 0 || t > 1) {
            throw new IllegalArgumentException("t must be in the range [0, 1]");
        }

        double ddx = 20 * x5 * Math.pow(t, 3) + 12 * x4 * Math.pow(t, 2) + 6 * x3 * t + 2 * x2;
        double ddy = 20 * y5 * Math.pow(t, 3) + 12 * y4 * Math.pow(t, 2) + 6 * y3 * t + 2 * y2;

        return new Translation2d(ddx, ddy);
    }
}