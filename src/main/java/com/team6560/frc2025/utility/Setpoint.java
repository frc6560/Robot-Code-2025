package com.team6560.frc2025.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Setpoint {
    public double x;
    public double y;
    public double theta;
    public double vx;
    public double vy;
    public double omega;

    public Setpoint(double x, double y, double theta, double vx, double vy, double omega){
        this.x = x;
        this.y = y;
        this.theta = theta;

        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    public Pose2d getSetpointPose(){
        return new Pose2d(x, y, new Rotation2d(theta));
    }

    public Translation2d getTranslation(){
        return new Translation2d(x, y);
    }

    public double getSpeed(){
        return Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    }

    
}
