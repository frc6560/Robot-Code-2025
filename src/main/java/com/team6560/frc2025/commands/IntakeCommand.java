package com.team6560.frc2025.commands;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;

import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCommand extends SequentialCommandGroup{
    Timer pipeGrabberTimer = new Timer();
    PickupLocations location;
    Pose2d targetPickupPose;

    public IntakeCommand(Wrist wrist, Elevator elevator, SwerveSubsystem swervedrive, PipeGrabber grabber, PickupLocations pickupLocation) {
        location = pickupLocation;

        getTargetPose();
        final Command driveToIntakePos = swervedrive.pathfindToPose(targetPickupPose, 0);
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

        super.addCommands(new ParallelCommandGroup(driveToIntakePos, deactuateElevator));
        super.addRequirements(elevator, swervedrive, grabber);
    }

    void getTargetPose(){
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        switch(location){
            case RIGHT:
                targetPickupPose = alliance == DriverStation.Alliance.Red ? new Pose2d(16.219, 7.347, Rotation2d.fromDegrees(55)) 
                                                                            : new Pose2d(1.12, 1.05, Rotation2d.fromDegrees(-125));
                break;
            case LEFT:
                targetPickupPose = alliance == DriverStation.Alliance.Red ? new Pose2d(15.925, 0.681, Rotation2d.fromDegrees(-125)) 
                                                                            : new Pose2d(1.12, 7, Rotation2d.fromDegrees(55));
                break;
            case TEST:
                targetPickupPose = new Pose2d(10, 7, Rotation2d.fromDegrees(120));
                break;
        }
    }
}
