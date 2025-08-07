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
            () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0 
                    && Math.abs(wrist.getWristAngle() + 240 - WristConstants.WristStates.PICKUP) < 8.0)
        );

        FunctionalCommand intakePiece = new FunctionalCommand(
            () -> {
                pipeGrabberTimer.start();
            },
            () -> {
                wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                grabber.runIntakeMaxSpeed();
            },
            (interrupted) -> {
                grabber.stop();
            },
            () -> pipeGrabberTimer.hasElapsed(0.5)
        );

        super.addCommands(new ParallelCommandGroup(driveToIntakePos, deactuateElevator), intakePiece);
        super.addRequirements(elevator, swervedrive, grabber);
    }

    void getTargetPose(){
        switch(location){
            case RIGHT_RED:
            targetPickupPose = new Pose2d(16.189, 7.165, Rotation2d.fromDegrees(-125));
                break;
            case LEFT_RED:
                targetPickupPose = new Pose2d();
                break;
            case RIGHT_BLUE:
                targetPickupPose = new Pose2d();
                break;
            case LEFT_BLUE:
                targetPickupPose = new Pose2d();
                break;
            case TEST:
                targetPickupPose = new Pose2d(11.644, 6.096, Rotation2d.fromDegrees(120));
                break;
        }
    }
}
