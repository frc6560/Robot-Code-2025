package com.team6560.frc2025.commands.automations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.HashMap;
import java.util.Set;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.Constants.DrivebaseConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;
import com.team6560.frc2025.utility.LimelightHelpers;
import com.team6560.frc2025.utility.Setpoint;

import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A command that automatically aligns the robot to a target pose, actuates the elevator and wrist, scores, and retracts.
 * This command is used for scoring at the reef on any level, from L1-L4.
 */
public class CoralScoreCommandFactory{
    // Paths
    private AutoAlignPath path;

    // Profiles
    private TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);
    private TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0); // The position is actually the error.
    private TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

    private TrapezoidProfile.Constraints translationConstraints;
    private TrapezoidProfile.Constraints rotationConstraint;
    private TrapezoidProfile translationProfile;
    private TrapezoidProfile rotationProfile;

    // Subsystems
    private SwerveSubsystem drivetrain;
    private Wrist wrist;
    private Elevator elevator;
    private PipeGrabber grabber;


    /** Constructor for our scoring command */
    public CoralScoreCommandFactory(Wrist wrist, Elevator elevator, PipeGrabber grabber, SwerveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.wrist = wrist;
        this.elevator = elevator;
        this.grabber = grabber;
    }
    double wristTarget;
    double elevatorTarget;
    Pose2d targetPrescore;

    // ---- SCORING COMMANDS ----

    public Command getScoreTeleop(ReefLevel level, ReefSide side){
        return Commands.defer(
            () -> Commands.sequence(
                Commands.runOnce(
                    () -> {
                        double wristTarget = getSuperstructureTargets(level).getFirst();
                        elevatorTarget = getSuperstructureTargets(level).getSecond();
                    }
                ).withTimeout(1.5),
                Commands.parallel(
                    alignToTagCommand(side),
                    getActuateCommand(elevatorTarget, wristTarget)
                ).withTimeout(3), getScoreCommand(), getDeactuationCommand()
            ), Set.of(drivetrain, wrist, elevator, grabber));
    }

    public Command getScoreAuto(ReefSide side, ReefIndex index, ReefLevel level){
        return Commands.defer(
            () -> Commands.sequence(
                Commands.runOnce(
                    () -> {
                        targetPrescore = setPrescoreTarget(index, side);
                        Pair<Double, Double> superstructureTargets = getSuperstructureTargets(level);
                        wristTarget = superstructureTargets.getFirst();
                        elevatorTarget = superstructureTargets.getSecond();
                    }
                ),
                getDriveToPrescore(targetPrescore),
                Commands.parallel(
                    alignToTagCommand(side),
                    getActuateCommand(elevatorTarget, wristTarget)
                ).withTimeout(1.2), getScoreCommand()
            ), Set.of(drivetrain, wrist, elevator, grabber));
    }


    // ---- SUB COMMANDS ----


    /** Runs a grabber intake during auto, which decreases wait time at the station */
    public Command getGrabberIntake(){
        return new RunCommand(
            () -> grabber.runIntakeMaxSpeed(),
            grabber).withTimeout(0.4)
            .andThen(() -> grabber.stop());
    }

    /** Drives close to our target pose during auto */
    public Command getDriveToPrescore(Pose2d targetPose){
        path = new AutoAlignPath(
            drivetrain.getPose(),
            targetPose,
            DrivebaseConstants.kMaxAutoVelocity,
            DrivebaseConstants.kMaxAutoAcceleration,
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha);
        final Command driveToPrescore = getFollowPath(path, DrivebaseConstants.kMaxAlignmentVelocity).until(
            () -> drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.3
        ).andThen(
            () -> drivetrain.driveFieldOriented(
                new ChassisSpeeds(
                    DrivebaseConstants.kMaxAlignmentVelocity * Math.cos(drivetrain.getHeading().getRadians()),
                    DrivebaseConstants.kMaxAlignmentVelocity * Math.sin(drivetrain.getHeading().getRadians()),
                    0
                )
            )
        ).until(() -> drivetrain.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.05);
        return driveToPrescore;
    }

    double xError;
    double yError;
    double thetaError;

    /** Uses the vector in camera space to align to the reef, since it's pretty close to the tag. 
     * TODOs: Find the final x target and y target. Handle units mismatch in tolerances. And make a lookup table.
     * @return A {@link} Command that aligns the robot to the target pose using Limelight data.
     */
    public Command alignToTagCommand(ReefSide side){
        String limelightName = (side == ReefSide.LEFT) ? "limelight-right" : "limelight-left";

        double taTarget = (side == ReefSide.LEFT) ? -1.0 : 20.35; // TODO: these need to be tuned. magic numbers for now because this sucks
        double txTarget = (side == ReefSide.LEFT) ? -1.0 : 0.96;

        double xTarget = 1.0 / Math.sqrt(taTarget); // TODO: possibly add a lookup table
        double yTarget = Math.tan(txTarget) * xTarget;
        // Filters for rotation
        LinearFilter filter = LinearFilter.movingAverage(5);
        Command driveToTagPose = Commands.runOnce(
            () -> {
                // This is also a great place to update our odometry
                drivetrain.updateOdometryWithVision();
            }
        ).andThen(Commands.run(
            () -> {
                System.out.println("running");
                // Translation
                double ta = LimelightHelpers.getTA(limelightName); 
                double xEstimate = 1.0 / Math.sqrt(ta); // TODO: possibly add a lookup table
                xError = xEstimate - xTarget;
                double xOutput = (-1) * drivetrain.getXOutput(xError);

                double tx = LimelightHelpers.getTX(limelightName);
                double yEstimate = Math.tan(tx) * xEstimate;
                yError = yEstimate - yTarget;
                double yOutput = (-1) * drivetrain.getYOutput(yError);

                // Rotation
                thetaError = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getRotation().getZ(); // this number might be changed.
                thetaError = filter.calculate(thetaError);
                double rotationOutput = drivetrain.getRotationOutput(thetaError);
                drivetrain.drive(
                    new ChassisSpeeds(
                        xOutput,
                        yOutput,
                        rotationOutput
                    )
                );
            }
        )).until(
            () -> Math.abs(xError) < 0.02 && Math.abs(yError) < 0.02
                    && Math.abs(thetaError) < 0.017
        );
        return driveToTagPose;
    }

    /** Actuates superstructure to our desired level */
    public Command getActuateCommand(double elevatorTarget, double wristTarget){
        final Command actuateToPosition = new FunctionalCommand(
            () -> {
            },
            () -> {
                elevator.setElevatorPosition(elevatorTarget);
                wrist.setMotorPosition(wristTarget);
            },
            (interrupted) -> {},
            () ->  Math.abs(elevator.getElevatorHeight() - elevatorTarget) < ElevatorConstants.kElevatorTolerance
        );
        return actuateToPosition;
    }

    /** Scores the piece */
    public Command getScoreCommand(){
        final Command dunkAndScore = new FunctionalCommand(
            () -> {
            },
            () -> {
                // Makes sure the drivetrain is entirely stopped before ejecting
                drivetrain.drive(
                    new ChassisSpeeds(0, 0, 0)
                );
                grabber.runGrabberOuttake();
            },
            (interrupted) -> {
                grabber.stop();
            },
            () -> false
        );
        return dunkAndScore.withTimeout(0.25);
    }

    /** Deactuates the superstructure in teleop for driver QOL */
    public Command getDeactuationCommand(){
        final Command deactuateSuperstructure = new FunctionalCommand(
                        () -> {
                        },
                        () -> { 
                            wrist.setMotorPosition(WristConstants.WristStates.STOW);
                            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                        },
                        (interrupted) -> {},
                        () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < ElevatorConstants.kElevatorTolerance) 
        );
        return deactuateSuperstructure.withTimeout(0.5);
    }


    // ---- HELPER METHODS ---- 


    /** Transforms red alliance poses to blue by reflecting around the center point of the field*/
    public Pose2d applyAllianceTransform(Pose2d pose){
        return new Pose2d(
            pose.getX() - 2 * (pose.getX() - 8.75),
            pose.getY() - 2 * (pose.getY() - 4.0),
            pose.getRotation().rotateBy(Rotation2d.fromDegrees(180))
        );
    }

    /** Sets the target for the robot, including target pose, elevator height, and arm angle */
    public Pose2d setPrescoreTarget(ReefIndex index, ReefSide side){
        DriverStation.Alliance alliance;
        if(!DriverStation.getAlliance().isPresent()){
            alliance = DriverStation.Alliance.Blue;
        }
        else alliance = DriverStation.getAlliance().get();

        int multiplier = (side == ReefSide.LEFT) ? -1 : 1;
        final double DISTANCE_FROM_TAG = 0.164;
        HashMap<ReefIndex, Pose2d> targetPoses = new HashMap<ReefIndex, Pose2d>();

        // All target poses are in meters
        targetPoses.put(ReefIndex.BOTTOM_RIGHT, new Pose2d(13.477, 2.691, Rotation2d.fromDegrees(300))); // wrong
        targetPoses.put(ReefIndex.FAR_RIGHT, new Pose2d(14.423, 3.721, Rotation2d.fromDegrees(0)));
        targetPoses.put(ReefIndex.TOP_RIGHT, new Pose2d(14.006, 5.056, Rotation2d.fromDegrees(60)));
        targetPoses.put(ReefIndex.TOP_LEFT, new Pose2d(12.640, 5.361, Rotation2d.fromDegrees(120))); 
        targetPoses.put(ReefIndex.FAR_LEFT, new Pose2d(11.693, 4.331, Rotation2d.fromDegrees(180))); // wrong
        targetPoses.put(ReefIndex.BOTTOM_LEFT, new Pose2d(12.111, 2.996, Rotation2d.fromDegrees(240)));

        // Puts a HashMap of all possible april tag positions. Notice this is viewed top down with the barge to the left.
        Pose2d aprilTagPose = targetPoses.get(index);

        Pose2d targetPose = new Pose2d( 
            aprilTagPose.getX() + (DISTANCE_FROM_TAG * Math.cos(aprilTagPose.getRotation().getRadians() + Math.PI/2) * multiplier),
            aprilTagPose.getY() + (DISTANCE_FROM_TAG * Math.sin(aprilTagPose.getRotation().getRadians() + Math.PI/2) * multiplier),
            aprilTagPose.getRotation()
        );

        if(alliance == DriverStation.Alliance.Blue){
            targetPose = applyAllianceTransform(targetPose);
        }

        // Applies pre score transform
        return new Pose2d(
            targetPose.getX() + 1 * Math.cos(targetPose.getRotation().getRadians()), 
            targetPose.getY() + 1 * Math.sin(targetPose.getRotation().getRadians()), 
            targetPose.getRotation()
        );
    }

    /** Returns wrist, elevator targets */
    public Pair<Double, Double> getSuperstructureTargets(ReefLevel level){
        double wristTarget;
        double elevatorTarget;
        switch (level) {
            case L1:
                wristTarget = WristConstants.WristStates.L1;
                elevatorTarget = ElevatorConstants.ElevatorStates.STOW;
                break;
    
            case L2:
                wristTarget = WristConstants.WristStates.L2;
                elevatorTarget = ElevatorConstants.ElevatorStates.L2;
                break;
    
            case L3:
                wristTarget = WristConstants.WristStates.L2;
                elevatorTarget = ElevatorConstants.ElevatorStates.L3;
                break;
    
            case L4: 
                wristTarget = WristConstants.WristStates.L4;
                elevatorTarget = ElevatorConstants.ElevatorStates.L4;
                break;
    
            default:
                wristTarget = WristConstants.WristStates.STOW;
                elevatorTarget = ElevatorConstants.ElevatorStates.STOW;
                break;
        }
        return new Pair<Double, Double>(wristTarget, elevatorTarget);
    }

    /** Helper method for following a straight trajectory with a trapezoidal profile */
    public Command getFollowPath(AutoAlignPath path, double finalVelocity){
        final Command followPath = new FunctionalCommand(
            () -> {
            // Resets profiles and states
            translationConstraints = new Constraints(path.maxVelocity, path.maxAcceleration);
            rotationConstraint = new Constraints(path.maxAngularVelocity, path.maxAngularAcceleration);

            translationProfile = new TrapezoidProfile(translationConstraints);
            rotationProfile = new TrapezoidProfile(rotationConstraint);

            translationalState.position = path.getDisplacement().getNorm();
            translationalState.velocity = MathUtil.clamp(((-1) * (drivetrain.getFieldVelocity().vxMetersPerSecond * path.getDisplacement().getX() 
                                                                + drivetrain.getFieldVelocity().vyMetersPerSecond * path.getDisplacement().getY())/ translationalState.position),
                                                                -path.maxVelocity,
                                                                0);

            targetTranslationalState.velocity = finalVelocity;

            targetRotationalState.position = path.endPose.getRotation().getRadians();
            rotationalState.position = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
            rotationalState.velocity = drivetrain.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond;
        },
            () -> {
                // Move.
                Setpoint newSetpoint = getNextSetpoint(path);
                drivetrain.followSegment(newSetpoint, path.endPose);
            },
            (interrupted) -> {},
            () -> drivetrain.getPose().getTranslation().getDistance(path.endPose.getTranslation()) < 0.02
            && Math.abs(drivetrain.getPose().getRotation().getRadians() - path.endPose.getRotation().getRadians()) < 0.017
        );
        return followPath;

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
