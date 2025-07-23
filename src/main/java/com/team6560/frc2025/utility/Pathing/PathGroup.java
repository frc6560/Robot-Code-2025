package com.team6560.frc2025.utility.Pathing;

import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//TODOS: fix rotation. 
// FIX EVERYTHING

public class PathGroup{
    public Path firstPath;
    public Path secondPath;

    public TrapezoidProfile.State startState;
    public TrapezoidProfile.State endState;

    public TrapezoidProfile.State startRotation;
    public TrapezoidProfile.State endRotation;

    public TrapezoidProfile profile;

    /** Creates a PathGroup object. TODO: refactor?*/
    public PathGroup(Path firstPath, Path secondPath, double maxVelocity, double maxAt, double maxOmega, double maxAlpha) {
        this.firstPath = firstPath;
        this.secondPath = secondPath;

        this.startState = new TrapezoidProfile.State(0, 0);
        this.endState = new TrapezoidProfile.State(firstPath.getArcLength() + secondPath.getArcLength(), 0);

        this.startRotation = new TrapezoidProfile.State(firstPath.getStartPose().getRotation().getRadians(), 0);

        // Note that we cannot use the original paths' profiles, as this would generate an unnecessary pause at the transition point.
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAt)
        );
    }


    /** We don't really need any of the other functions. A modified calculation function. */
    public Setpoint calculate(double t, TrapezoidProfile.State translation, TrapezoidProfile.State rotation,
                            double currentRotation){
        // Translation
        TrapezoidProfile.State translationalSetpoint = profile.calculate(t, startState, endState);

        double curvature;
        Translation2d normalizedVelocity;
        Translation2d translationalTarget;
        double timeParam;

        if(translationalSetpoint.position < firstPath.getArcLength()){
            timeParam = firstPath.getTimeForArcLength(translationalSetpoint.position);
            translationalTarget = firstPath.calculatePosition(timeParam);

            curvature = firstPath.getCurvature(timeParam);
            normalizedVelocity = firstPath.getNormalizedVelocityVector(timeParam);
        } else {
            timeParam = secondPath.getTimeForArcLength(translationalSetpoint.position - firstPath.getArcLength());
            translationalTarget = secondPath.calculatePosition(timeParam);

            curvature = secondPath.getCurvature(timeParam);
            normalizedVelocity = secondPath.getNormalizedVelocityVector(timeParam);
        }

        // fix rotation later
        double thetaTarget = 0;
        // return new Setpoint(translationalTarget.getX(),
        //                     translationalTarget.getY(), 
        //                     thetaTarget, 
        //                     normalizedVelocity.getX() * velocityMultiplier, 
        //                     normalizedVelocity.getY() * velocityMultiplier,
        //                     0);
        return null;
    }
}