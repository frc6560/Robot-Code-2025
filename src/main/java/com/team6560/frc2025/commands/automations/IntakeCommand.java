package com.team6560.frc2025.commands.automations;

import com.team6560.frc2025.Constants.DrivebaseConstants;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;
import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public IntakeCommand(Wrist wrist, Elevator elevator, SwerveSubsystem drivetrain, String pathFileName) {
        Command pathCommand = drivetrain.getAutonomousCommand(pathFileName);
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

    // void getTargetPose(){
    //     DriverStation.Alliance alliance;
    //     if(!DriverStation.getAlliance().isPresent()){
    //         alliance = DriverStation.Alliance.Blue;
    //     }
    //     else alliance = DriverStation.getAlliance().get();
    //     switch(location){
    //         case RIGHT:
    //             targetPickupPose = alliance == DriverStation.Alliance.Red ? new Pose2d(16.219+0.02, 7.347+0.02, Rotation2d.fromDegrees(55)) 
    //                                                                         : new Pose2d(1.585-0.02, 0.727-0.02, Rotation2d.fromDegrees(-125));
    //             break;
    //         case LEFT:
    //             targetPickupPose = alliance == DriverStation.Alliance.Red ? new Pose2d(15.925+0.02, 0.681-0.02, Rotation2d.fromDegrees(-55)) 
    //                                                                         : new Pose2d(1.652-0.02, 7.347+0.02, Rotation2d.fromDegrees(125));
    //             break;
    //         case TEST:
    //             targetPickupPose = new Pose2d(14.339, 6.794, Rotation2d.fromDegrees(55));
    //             break;
    //     }
    // }
}
