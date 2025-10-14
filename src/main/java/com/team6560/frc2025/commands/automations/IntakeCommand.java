package com.team6560.frc2025.commands.automations;

import com.team6560.frc2025.Constants.DrivebaseConstants;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCommand extends SequentialCommandGroup{
    PickupLocations location;
    Pose2d targetPickupPose;

    // Note that this needs a second auto align, solely because rotation needs to be calculated differently.
    TrapezoidProfile translationProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            DrivebaseConstants.kMaxAutoVelocity,
            DrivebaseConstants.kMaxAutoVelocity) // max velocity and acceleration
    );
    TrapezoidProfile rotationProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            DrivebaseConstants.kMaxOmega,
            DrivebaseConstants.kMaxAlpha) // max angular velocity and acceleration
    );

    TrapezoidProfile.State translationState = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State rotationState = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State translationTarget = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State rotationTarget =  new TrapezoidProfile.State(0, 0);

    AutoAlignPath path;

    public IntakeCommand(Wrist wrist, Elevator elevator, SwerveSubsystem drivetrain, PickupLocations pickupLocation) {
        location = pickupLocation;

        getTargetPose();

        Command pathCommand = new FunctionalCommand(
            () -> {
                path = new AutoAlignPath(
                    drivetrain.getPose(),
                    targetPickupPose,
                    DrivebaseConstants.kMaxAutoVelocity,
                    DrivebaseConstants.kMaxAutoAcceleration,
                    DrivebaseConstants.kMaxOmega,
                    DrivebaseConstants.kMaxAlpha
                );
                translationState.position = path.getDisplacement().getNorm();
                rotationState.position = path.startPose.getRotation().getRadians();
                rotationTarget.position = path.endPose.getRotation().getRadians();
            },
            () -> {
                // trans
                State translationSetpoint = translationProfile.calculate(0.02, translationState, translationTarget);
                translationState.position = translationSetpoint.position;
                translationState.velocity = translationSetpoint.velocity;

                // only activate rotation at 4.5m from target
                if(drivetrain.getPose().getTranslation().getDistance(targetPickupPose.getTranslation()) < 3.2){
                    // rot wraparound calculations
                    double rotationalPose = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
                    double goalRotation = rotationTarget.position;
                    double angularError = MathUtil.angleModulus(goalRotation - rotationalPose);

                    rotationTarget.position = rotationalPose + angularError;
                    double setpointError = MathUtil.angleModulus(rotationState.position - rotationalPose);
                    rotationState.position = rotationalPose + setpointError;
                    // rot
                    State rotationalSetpoint = rotationProfile.calculate(0.02, rotationState, rotationTarget);
                    rotationState.position = rotationalSetpoint.position;
                    rotationState.velocity = rotationalSetpoint.velocity;
                }

                // arc length parametrization for a line
                Translation2d interpolatedTranslation = path.endPose
                    .getTranslation()
                    .interpolate(path.startPose.getTranslation(), 
                    translationSetpoint.position / path.getDisplacement().getNorm());

                Setpoint targetSetpoint =  new Setpoint(
                    interpolatedTranslation.getX(),
                    interpolatedTranslation.getY(),
                    rotationState.position,
                    path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity, 
                    path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity,
                    rotationState.velocity
                );
                drivetrain.followSegment(targetSetpoint, targetPickupPose, true);
            },
            (interrupted) -> {
                drivetrain.drive(new ChassisSpeeds(0, 0, 0));
            },
            () -> drivetrain.getPose().getTranslation().getDistance(path.endPose.getTranslation()) < 0.1
            && Math.abs(drivetrain.getPose().getRotation().getRadians() - path.endPose.getRotation().getRadians()) < 0.1

        );
        FunctionalCommand deactuateElevator = new FunctionalCommand(
            () -> {
            },
            () -> {
                wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
            },
            (interrupted) -> {},
            () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0)
        );

        super.addCommands(new ParallelCommandGroup(pathCommand, deactuateElevator));
        super.addRequirements(elevator, drivetrain, wrist);
    }

    void getTargetPose(){
        DriverStation.Alliance alliance;
        if(!DriverStation.getAlliance().isPresent()){
            alliance = DriverStation.Alliance.Blue;
        }
        else alliance = DriverStation.getAlliance().get();
        switch(location){
            case RIGHT:
                targetPickupPose = alliance == DriverStation.Alliance.Red ? new Pose2d(16.219+0.02, 7.347+0.02, Rotation2d.fromDegrees(55)) 
                                                                            : new Pose2d(1.12, 1.05, Rotation2d.fromDegrees(-125));
                break;
            case LEFT:
                targetPickupPose = alliance == DriverStation.Alliance.Red ? new Pose2d(15.925, 0.681, Rotation2d.fromDegrees(-125)) 
                                                                            : new Pose2d(1.12, 7, Rotation2d.fromDegrees(55));
                break;
            case TEST:
                targetPickupPose = new Pose2d(14.339, 6.794, Rotation2d.fromDegrees(55));
                break;
        }
    }
}
