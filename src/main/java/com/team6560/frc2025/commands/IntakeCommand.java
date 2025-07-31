package com.team6560.frc2025.commands;

import java.nio.channels.Pipe;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;

import com.team6560.frc2025.Constants.ElevatorConstants.ElevatorStates;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeCommand extends SequentialCommandGroup{
    private Elevator elevator;
    private SwerveSubsystem swervedrive;
    private PipeGrabber grabber;
    Timer pipeGrabberTimer = new Timer();

    public IntakeCommand(Elevator elevator, SwerveSubsystem swervedrive, PipeGrabber grabber, Pose2d targetPickupPose){
        this.elevator = elevator;
        this.swervedrive = swervedrive;
        this.grabber = grabber;

        final Command driveToIntakePos = swervedrive.pathfindToPose(targetPickupPose);
        FunctionalCommand deactuateElevator = new FunctionalCommand(
            () -> {
            },
            () -> {
                System.out.println("hi");
                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
            },
            (interrupted) -> {},
            () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0)
        );

        FunctionalCommand intakePiece = new FunctionalCommand(
            () -> {
                pipeGrabberTimer.start();
            },
            () -> {
                grabber.runIntakeMaxSpeed();
            },
            (interrupted) -> {},
            () -> pipeGrabberTimer.hasElapsed(0.5)
        );

        super.addCommands(new ParallelCommandGroup(driveToIntakePos, deactuateElevator), intakePiece);
        super.addRequirements(elevator, swervedrive, grabber);
    }
}
