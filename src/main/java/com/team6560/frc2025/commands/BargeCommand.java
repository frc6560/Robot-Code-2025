package com.team6560.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2025.subsystems.BallGrabber;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;
import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;

public class BargeCommand extends Command {

    // Motion Profile Constants
    final double MAX_VELOCITY = 2.7;
    final double MAX_ACCELERATION = 2.2;
    final double MAX_OMEGA = Math.toRadians(540);
    final double MAX_ALPHA = Math.toRadians(720);

    // Tolerances to determine when the command has successfully finished
    final double TRANSLATIONAL_TOLERANCE = 0.05; // meters
    final double ROTATIONAL_TOLERANCE = 0.05;  // radians

    // Poses
    private Pose2d targetPose;

    private AutoAlignPath alignPath;

    // Profiles for smooth motion generation
    private final TrapezoidProfile translationProfile;
    private final TrapezoidProfile rotationProfile;

    private TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0); // Goal: 0 distance, 0 velocity
    private TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

    // Subsystems
    private final SwerveSubsystem drivetrain;
    private final Wrist wrist;
    private final Elevator elevator;
    private final BallGrabber grabber;

    public BargeCommand(SwerveSubsystem drivetrain, Wrist wrist, Elevator elevator, BallGrabber grabber) {
        this.drivetrain = drivetrain;
        this.wrist = wrist;
        this.elevator = elevator;
        this.grabber = grabber;
        
        // Initialize the profile objects with their constraints in the constructor
        this.translationProfile = new TrapezoidProfile(new Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        this.rotationProfile = new TrapezoidProfile(new Constraints(MAX_OMEGA, MAX_ALPHA));

        // Declare subsystem requirements
        addRequirements(drivetrain, wrist, elevator, grabber);
    }

   
    @Override
    public void initialize() {
        
        boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
        this.targetPose = getBargeTarget(isBlue);

        this.alignPath = new AutoAlignPath(drivetrain.getPose(), targetPose, MAX_VELOCITY, MAX_ACCELERATION, MAX_OMEGA, MAX_ALPHA);

        translationalState.position = alignPath.getDisplacement().getNorm();
        translationalState.velocity = 0;

        targetRotationalState.position = targetPose.getRotation().getRadians();
        rotationalState.position = drivetrain.getPose().getRotation().getRadians();
        rotationalState.velocity = drivetrain.getRobotVelocity().omegaRadiansPerSecond;
    }

    /**
     * Called REPEATEDLY every 20ms while the command is running.
     * This is the "work" phase where we calculate and command the next small movement.
     */
    @Override
    public void execute() {
        
        Setpoint nextSetpoint = getNextSetpoint(alignPath);
        
        // Command the drivetrain to move towards that setpoint
        drivetrain.followSegment(nextSetpoint);
    }

    @Override
    public boolean isFinished() {
        boolean translationDone = translationalState.position < TRANSLATIONAL_TOLERANCE;
        
        double rotationError = targetPose.getRotation().getRadians() - drivetrain.getPose().getRotation().getRadians();
        boolean rotationDone = Math.abs(MathUtil.angleModulus(rotationError)) < ROTATIONAL_TOLERANCE;

        return translationDone && rotationDone;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }


    public Pose2d getBargeTarget(Boolean alliance) {
        if (alliance) {
            return new Pose2d(7.8, MathUtil.clamp(drivetrain.getPose().getY(), 4.7, 7.4), Rotation2d.fromDegrees(180));
        } else {
            return new Pose2d(7.8, MathUtil.clamp(drivetrain.getPose().getY(), 0.475, 3.2), Rotation2d.fromDegrees(180));
        }
    }

    /**
     * Calculates the next Setpoint (target state) for the robot to follow for one 20ms cycle.
     * This is the core of the motion profiling logic.
     * @param path The AutoAlignPath the robot is currently following.
     * @return A Setpoint object containing the target pose, and target velocities.
     */
    public Setpoint getNextSetpoint(AutoAlignPath path) {
        State translationSetpoint = translationProfile.calculate(0.02, translationalState, targetTranslationalState);
        translationalState.position = translationSetpoint.position;
        translationalState.velocity = translationSetpoint.velocity;
        double rotationalPose = drivetrain.getPose().getRotation().getRadians();
        double goalRotation = targetRotationalState.position;
        double angularError = MathUtil.angleModulus(goalRotation - rotationalPose);
        targetRotationalState.position = rotationalPose + angularError;
        double setpointError = MathUtil.angleModulus(rotationalState.position - rotationalPose);
        rotationalState.position = rotationalPose + setpointError;
        State rotationalSetpoint = rotationProfile.calculate(0.02, rotationalState, targetRotationalState);
        rotationalState.position = rotationalSetpoint.position;
        rotationalState.velocity = rotationalSetpoint.velocity;

        Translation2d interpolatedTranslation = path.endPose
            .getTranslation()
            .interpolate(path.startPose.getTranslation(),
                translationSetpoint.position / path.getDisplacement().getNorm());

        // Create the final Setpoint object with all target values
        return new Setpoint(
            interpolatedTranslation.getX(),
            interpolatedTranslation.getY(),
            rotationalState.position, // The target angle
            path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity, // Target X velocity
            path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity, // Target Y velocity
            rotationalState.velocity // Target angular velocity
        );
    }
} 
//I forgor to save last time :Skull EMoji: