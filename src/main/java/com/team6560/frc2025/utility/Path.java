package com.team6560.frc2025.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A stub, currently only implements a linear path. Enough for my purposes for now*/
public class Path {
    public final Pose2d startPose;
    public final Pose2d endPose;
    public final double maxVelocity;
    public final double maxAcceleration;

    public final double maxAngularVelocity;
    public final double maxAngularAcceleration;

    public Path( Pose2d startPose,Pose2d endPose, double maxVelocity,double maxAcceleration, 
                 double maxAngularVelocity, double maxAngularAcceleration) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    public Translation2d getDisplacement() {
        return startPose.getTranslation().minus(endPose.getTranslation());
    }

    public double getRotationError() {
        return startPose.getRotation().minus(endPose.getRotation()).getDegrees();
    }
}