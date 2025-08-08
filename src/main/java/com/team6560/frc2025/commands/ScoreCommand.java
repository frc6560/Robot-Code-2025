package com.team6560.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.HashMap;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;
import com.team6560.frc2025.utility.Setpoint;

import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command that automatically aligns the robot to a target pose, actuates the elevator and wrist, scores, and retracts.
 * This command is used for scoring at the reef on any level, from L1-L4.
 */
public class ScoreCommand extends SequentialCommandGroup {
    // Consts
    final double E_TOLERANCE = 1.0;
    final double W_TOLERANCE = 8.0;

    final double MAX_VELOCITY = 1.8; // originally 1.8
    final double MAX_ACCELERATION = 1.5; // originally 1.5
    final double MAX_OMEGA = Math.toRadians(540);
    final double MAX_ALPHA = Math.toRadians(720);

    // Poses
    private Pose2d targetPose;

    // Paths
    private AutoAlignPath startPath;
    private AutoAlignPath endPath;

    // Profiles
    private TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0); // The position is actually the error.
    private TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile.Constraints translationConstraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    private TrapezoidProfile.Constraints rotationConstraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);
    private TrapezoidProfile translationProfile = new TrapezoidProfile(translationConstraints);
    private TrapezoidProfile rotationProfile = new TrapezoidProfile(rotationConstraints);

    // Subsystems
    private SwerveSubsystem drivetrain;
    private Wrist wrist;
    private Elevator elevator;
    private PipeGrabber grabber;

    // Levels
    ReefSide side;
    ReefIndex location;
    ReefLevel level;

    // Targets
    private double elevatorTarget;
    private double wristTarget;
    private double wristOffset;

    // A timer :(
    Timer grabberTimer = new Timer();

    /** A constructor to score at a given level... in teleoperated mode.*/
    public ScoreCommand(Wrist wrist, Elevator elevator, PipeGrabber grabber, SwerveSubsystem drivetrain, 
                            ReefSide side, ReefIndex location, ReefLevel level, boolean isAuto) {

        this.drivetrain = drivetrain;
        this.wrist = wrist;
        this.elevator = elevator;
        this.grabber = grabber;

        this.side = side;
        this.location = location;
        this.level = level;

        setTargets();

        if(isAuto){
            super.addCommands(getPathfindToPose(), new ParallelCommandGroup(getDriveInCommand(), getActuateCommand()), getScoreCommand(), getPartialDeactuationCommand());
        }
        else{
            super.addCommands(getPathfindToPose(), new ParallelCommandGroup(getDriveInCommand(), getActuateCommand()), getScoreCommand(), getFullDeactuationCommand());
        }
        super.addRequirements(wrist, elevator, grabber, drivetrain);
    }

    /** Gets a pathfinding command */
    public Command getPathfindToPose(){
        final Command pathfindToPose = drivetrain.pathfindToPose(getPrescore(targetPose), 0.9); // originally 0.9
        return pathfindToPose;
    }

    /** Gets an auto align command */
    public Command getDriveInCommand(){
        final Command driveIn = new FunctionalCommand(
            () -> {
                // Computes the necessary states for the translational and rotational profiles for our path.
                startPath = new AutoAlignPath(
                    drivetrain.getPose(),
                    targetPose,
                    MAX_VELOCITY, 
                    MAX_ACCELERATION, 
                    MAX_OMEGA,
                    MAX_ALPHA);


                endPath = new AutoAlignPath(
                    targetPose,
                    getPrescore(targetPose),
                    MAX_VELOCITY, 
                    MAX_ACCELERATION, 
                    MAX_OMEGA,
                    MAX_ALPHA);
                translationalState.position = startPath.getDisplacement().getNorm();
                translationalState.velocity = MathUtil.clamp(((-1) * (drivetrain.getFieldVelocity().vxMetersPerSecond * startPath.getDisplacement().getX() 
                                                            + drivetrain.getFieldVelocity().vyMetersPerSecond * startPath.getDisplacement().getY())/ translationalState.position),
                                                            -startPath.maxVelocity,
                                                            0);


                targetRotationalState.position = startPath.endPose.getRotation().getRadians();
                rotationalState.position = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
                rotationalState.velocity = drivetrain.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond;
            },
            () -> {
                if((translationalState.position < 0.03) && (Math.abs(rotationalState.position - targetRotationalState.position) < 0.05)){
                    drivetrain.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
                }
                else {
                    Setpoint newSetpoint = getNextSetpoint(startPath);
                    drivetrain.followSegment(newSetpoint);
                }
            },
            (interrupted) -> {},
            () -> (translationalState.position < 0.05) && Math.abs(rotationalState.position - targetRotationalState.position) < 0.05
        );
        return driveIn;
    }

    /** Gets an actuate to position command */
    public Command getActuateCommand(){
        final Command actuateToPosition = new FunctionalCommand(
            () -> {
            },
            () -> {
                if(drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.6){
                    elevator.setElevatorPosition(elevatorTarget);
                    wrist.setMotorPosition(wristTarget);
                }
            },
            (interrupted) -> {},
            () ->  Math.abs(elevator.getElevatorHeight() - elevatorTarget) < E_TOLERANCE 
            && Math.abs(wrist.getWristAngle() + 240 - wristTarget) < W_TOLERANCE
        );
        return actuateToPosition;
    }

    /** Gets a command to score the game piece*/
    public Command getScoreCommand(){
        final Command dunkAndScore = new FunctionalCommand(
            () -> {
            },
            () -> {
                wrist.setMotorPosition(wristTarget - wristOffset);
                if(Math.abs(wrist.getWristAngle() + 240 - (wristTarget - wristOffset)) < W_TOLERANCE){ 
                    grabberTimer.start();
                    grabber.runGrabberOuttake();
                }
            },
            (interrupted) -> {
                grabber.stop();
            },
            () -> grabberTimer.hasElapsed(0.25)
        );
        return dunkAndScore;
    }

    /** Gets a deactuation command for teleop */
    public Command getFullDeactuationCommand(){
        final Command backUp = new FunctionalCommand(
                        () -> {
                            // Resets all trapezoidal profiles
                            translationalState.position = endPath.getDisplacement().getNorm();
                            translationalState.velocity = 0;

                            targetRotationalState.position = endPath.endPose.getRotation().getRadians();
                            rotationalState.position = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
                            rotationalState.velocity = 0;
                        },
                        () -> { 
                            wrist.setMotorPosition(WristConstants.WristStates.STOW);
                            if(Math.abs(wrist.getWristAngle() + 240 - WristConstants.WristStates.STOW) < W_TOLERANCE){ 
                                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);

                                Setpoint newSetpoint = getNextSetpoint(endPath);
                                drivetrain.followSegment(newSetpoint);
                            }
                        },
                        (interrupted) -> {},
                        () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0) 
        );
        return backUp;
    }

    /** Gets a command to retract the wrist only. Used in autonomous. */
    public Command getPartialDeactuationCommand(){
        final Command retractWrist = new FunctionalCommand(
            () -> {
            },
            () -> {
                wrist.setMotorPosition(WristConstants.WristStates.STOW);
            },
            (interrupted) -> {},
            () -> 
                Math.abs(wrist.getWristAngle() + 240 - WristConstants.WristStates.STOW) < W_TOLERANCE
            
        );
        return retractWrist;
    }

    /** Gets the prescore for a specific Pose2d*/
    public Pose2d getPrescore(Pose2d targetPose){
        return new Pose2d(
            targetPose.getX() + 0.75 * Math.cos(targetPose.getRotation().getRadians()),
            targetPose.getY() + 0.75 * Math.sin(targetPose.getRotation().getRadians()),
            targetPose.getRotation()
        );
    }

    /** Sets the target for the robot, including target pose, elevator height, and arm angle */
    public void setTargets(){
        int multiplier = (side == ReefSide.LEFT) ? -1 : 1;
        final double DISTANCE_FROM_TAG = 0.164;

        // Puts a HashMap of all possible april tag positions. Notice that this is viewed bottom up, with y increasing.
        HashMap<ReefIndex, Pose2d> targetPoses = new HashMap<>();
        targetPoses.put(ReefIndex.BOTTOM_RIGHT, new Pose2d(13.426, 2.727, Rotation2d.fromDegrees(300)));
        targetPoses.put(ReefIndex.FAR_RIGHT, new Pose2d(14.344, 3.722, Rotation2d.fromDegrees(0)));
        targetPoses.put(ReefIndex.TOP_RIGHT, new Pose2d(13.991, 5.025, Rotation2d.fromDegrees(60)));
        targetPoses.put(ReefIndex.TOP_LEFT, new Pose2d(12.635, 5.377, Rotation2d.fromDegrees(120))); 
        targetPoses.put(ReefIndex.FAR_LEFT, new Pose2d(11.784, 4.339, Rotation2d.fromDegrees(180)));
        targetPoses.put(ReefIndex.BOTTOM_LEFT, new Pose2d(12.148, 3.064, Rotation2d.fromDegrees(240)));

        Pose2d aprilTagPose = targetPoses.get(location);

        targetPose = new Pose2d( 
            aprilTagPose.getX() + (DISTANCE_FROM_TAG * Math.cos(aprilTagPose.getRotation().getRadians() + Math.PI/2) * multiplier),
            aprilTagPose.getY() + (DISTANCE_FROM_TAG * Math.sin(aprilTagPose.getRotation().getRadians() + Math.PI/2) * multiplier),
            aprilTagPose.getRotation()
        );
        
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
            targetPose = applyAllianceTransform(targetPose);
        }


        // Sets subsystem targets
        switch (level) {
            case L1:
                wristTarget = WristConstants.WristStates.L1;
                wristOffset = WristConstants.WristStates.L1Offset;
                elevatorTarget = ElevatorConstants.ElevatorStates.STOW;
                break;

            case L2:
                wristTarget = WristConstants.WristStates.L2;
                wristOffset = WristConstants.WristStates.L2Offset;
                elevatorTarget = ElevatorConstants.ElevatorStates.L2;
                break;

            case L3:
                wristTarget = WristConstants.WristStates.L2;
                wristOffset = WristConstants.WristStates.L2Offset;
                elevatorTarget = ElevatorConstants.ElevatorStates.L3;
                break;

            case L4: 
                wristTarget = WristConstants.WristStates.L4;
                wristOffset = WristConstants.WristStates.L4Offset;
                elevatorTarget = ElevatorConstants.ElevatorStates.L4;
                break;

            default:
                wristTarget = WristConstants.WristStates.STOW;
                wristOffset = 0;
                elevatorTarget = ElevatorConstants.ElevatorStates.STOW;
                break;
        }
    }

    /** Transforms red alliance poses to blue by reflecting around the center point of the field*/
    public Pose2d applyAllianceTransform(Pose2d pose){
        return new Pose2d(
            pose.getX() - 2 * (pose.getX() - 8.75),
            pose.getY() - 2 * (pose.getY() - 4.0),
            pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );
    }

    /** Gets a setpoint for the robot PID to follow.
     * @return A Setpoint object representing the next target robot state on a certain auto align path.
     */
    public Setpoint getNextSetpoint(AutoAlignPath path){
        // trans
        State translationSetpoint = translationProfile.calculate(0.02, translationalState, targetTranslationalState);
        translationalState.position = translationSetpoint.position;
        translationalState.velocity = translationSetpoint.velocity;

        // rot wraparound calculations
        double rotationalPose = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
        double goalRotation = targetRotationalState.position;
        double angularError = MathUtil.angleModulus(goalRotation - rotationalPose);

        targetRotationalState.position = rotationalPose + angularError;
        double setpointError = MathUtil.angleModulus(rotationalState.position - rotationalPose);
        rotationalState.position = rotationalPose + setpointError;
        // rot
        State rotationalSetpoint = rotationProfile.calculate(0.02, rotationalState, targetRotationalState);
        rotationalState.position = rotationalSetpoint.position;
        rotationalState.velocity = rotationalSetpoint.velocity;

        // arc length parametrization for a line
        Translation2d interpolatedTranslation = path.endPose
            .getTranslation()
            .interpolate(path.startPose.getTranslation(), 
            translationSetpoint.position / path.getDisplacement().getNorm());

        return new Setpoint(
            interpolatedTranslation.getX(),
            interpolatedTranslation.getY(),
            rotationalState.position,
            path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity, 
            path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity,
            rotationalState.velocity
        );
    }
}
