package com.team6560.frc2025.utility.Pathing;

import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


public class PathGroup{
    public Path firstPath;
    public Path secondPath;

    public TrapezoidProfile.State currentState;
    public TrapezoidProfile.State endState;

    public TrapezoidProfile.State startRotation;
    public TrapezoidProfile.State currentRotationState;
    public TrapezoidProfile.State endRotation;

    public TrapezoidProfile profile;
    public TrapezoidProfile rotationProfile;

    /** Creates a PathGroup object. TODO: refactor?*/
    public PathGroup(Path firstPath, Path secondPath, double maxVelocity, double maxAt, double maxOmega, double maxAlpha) {
        this.firstPath = firstPath;
        this.secondPath = secondPath;

        this.currentState = new TrapezoidProfile.State(0, firstPath.getStartVelocity());
        this.endState = new TrapezoidProfile.State(firstPath.getArcLength() + secondPath.getArcLength(), secondPath.getEndVelocity());

        this.startRotation = new TrapezoidProfile.State(firstPath.getStartPose().getRotation().getRadians(), 0);
        this.currentRotationState = new TrapezoidProfile.State(firstPath.getStartPose().getRotation().getRadians(), 0);
        this.endRotation = new TrapezoidProfile.State(secondPath.getEndPose().getRotation().getRadians(), 0);

        // Note that we cannot use the original paths' profiles, as this would generate an unnecessary pause at the transition point.
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAt)
        );
        this.rotationProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxOmega, maxAlpha)
        );
    }


    /** We don't really need any of the other functions. A modified calculation function. */
    public Setpoint calculate(double currentRotation){
        // Translation
        TrapezoidProfile.State translationalSetpoint = profile.calculate(0.02, currentState, endState);

        Translation2d normalizedVelocity;
        double timeParam;
        Translation2d translationalTarget;

        currentState.position = translationalSetpoint.position;
        currentState.velocity = translationalSetpoint.velocity;

        // Rotation
        double rotationalPose = currentRotation;
        double errorToGoal = MathUtil.angleModulus(endRotation.position - rotationalPose);
        double errorToSetpoint = MathUtil.angleModulus(startRotation.position - rotationalPose);

        // gets rid of mod 2pi issues
        endRotation.position = rotationalPose + errorToGoal;
        currentRotationState.position = rotationalPose + errorToSetpoint;

        // finally computes next rotation state 
        State rotationalSetpoint = rotationProfile.calculate(0.02, currentRotationState, endRotation);
        currentRotationState.position = rotationalSetpoint.position;
        currentRotationState.velocity = rotationalSetpoint.velocity;

        // interpolation
        if(translationalSetpoint.position <= firstPath.getArcLength()-1E-6){
            timeParam = firstPath.getTimeForArcLength(translationalSetpoint.position);
            normalizedVelocity = firstPath.getNormalizedVelocityVector(timeParam);
            translationalTarget = firstPath.calculatePosition(timeParam);
        } else {
            timeParam = secondPath.getTimeForArcLength(translationalSetpoint.position - firstPath.getArcLength());
            normalizedVelocity = secondPath.getNormalizedVelocityVector(timeParam);
            translationalTarget = secondPath.calculatePosition(timeParam);
        }

        return new Setpoint(translationalTarget.getX(),
                            translationalTarget.getY(), 
                            rotationalSetpoint.position, 
                            normalizedVelocity.getX() * translationalSetpoint.velocity, 
                            normalizedVelocity.getY() * translationalSetpoint.velocity,
                            rotationalSetpoint.velocity);
    }
}