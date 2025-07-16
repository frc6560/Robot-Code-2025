package com.team6560.frc2025.utility.Pathing;

import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class PathGroup{
    public Path firstPath;
    public Path secondPath;

    public TrapezoidProfile.State startState;
    public TrapezoidProfile.State endState;

    public TrapezoidProfile profile;

    public final double MAX_A;

    /** Creates a PathGroup object. TODO: refactor?*/
    public PathGroup(Path firstPath, Path secondPath, double maxVelocity, double maxAt, double staticCof) {
        this.firstPath = firstPath;
        this.secondPath = secondPath;

        this.startState = new TrapezoidProfile.State(0, 0);
        this.endState = new TrapezoidProfile.State(firstPath.getArcLength() + secondPath.getArcLength(), 0);

        // Note that we cannot use the original paths' profiles, as this would generate an unnecessary pause at the transition point.
        this.profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity, maxAt)
        );

        this.MAX_A = staticCof * 9.81;
    }


    /** We don't really need any of the other functions. A modified calculation function. */
    public Setpoint calculate(double t, double currentRotation){
        // Translation
        TrapezoidProfile.State translationalSetpoint = profile.calculate(t, startState, endState);
        TrapezoidProfile.State previousState = profile.calculate(t - 0.02, startState, endState);

        // Calculates max ac based upon at and takes the maximum velocity possible.
        double AT = (translationalSetpoint.velocity - previousState.velocity) / 0.02; // change in velocity over time
        AT = MathUtil.clamp(AT, -MAX_A, MAX_A); // clamps to max acceleration
        double MAX_AC = Math.sqrt(Math.pow (MAX_A, 2) - Math.pow(AT, 2)); // calculates max centripetal acceleration

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
        double velocityMultiplier = Math.min(translationalSetpoint.velocity, Math.sqrt(MAX_AC / curvature + 1E-6));

        // fix rotation later
        double thetaTarget = 0;
        return new Setpoint(translationalTarget.getX(),
                            translationalTarget.getY(), 
                            thetaTarget, 
                            normalizedVelocity.getX() * velocityMultiplier, 
                            normalizedVelocity.getY() * velocityMultiplier,
                            0);
    }
}