package com.team6560.frc2025.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A stub, currently only implements a linear path. Enough for my purposes for now*/
public class AutoAlignPath {
    public final Pose2d startPose;
    public final Pose2d endPose;
    public final double maxVelocity;
    public final double maxAcceleration;

    public final double maxAngularVelocity;
    public final double maxAngularAcceleration;

    public AutoAlignPath( Pose2d startPose,Pose2d endPose, double maxVelocity,double maxAcceleration, 
                    double maxAngularVelocity, double maxAngularAcceleration) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;
    }

    public Translation2d getDisplacement() {
        return endPose.getTranslation().minus(startPose.getTranslation());
    }
    
    public Translation2d getNormalizedDisplacement() {
        Translation2d displacement = getDisplacement();
        double length = displacement.getNorm();
        return length == 0 ? new Translation2d(0, 0) : displacement.div(length + 0.001);
    }

    public double getRotationError() {
        return startPose.getRotation().minus(endPose.getRotation()).getDegrees();
    }
}