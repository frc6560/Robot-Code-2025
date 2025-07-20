package com.team6560.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.team6560.frc2025.ManualControls;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;

public class Score extends SequentialCommandGroup{

    public Score(Wrist wrist, Elevator elevator, PipeGrabber grabber, ManualControls controls, double targetWristAngle, double targetElevatorHeight) {

        Timer ejectTimer = new Timer();
        Timer downTimer = new Timer();

        double wristTarget = targetWristAngle;
        double elevatorTarget = targetElevatorHeight;

        double E_TOLERANCE = 1.0;
        double W_TOLERANCE = 8.0;

        final Command mechanismUp = new FunctionalCommand(
                    () -> {
                    },
                    () -> {
                        elevator.setElevatorPosition(elevatorTarget);
                        wrist.setMotorPosition(wristTarget);
                        if(Math.abs(elevator.getElevatorHeight() - elevatorTarget) < E_TOLERANCE && 
                           Math.abs(wrist.getWristAngle() - wristTarget) < W_TOLERANCE) {
                            grabber.runGrabberOuttakeMaxSpeed();
                        }
                    },
                    (interrupted) -> {},
                    () -> !grabber.hasGamePiece()
                    );

        final Command mechanismDown = new FunctionalCommand(
                        () -> {
                            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                            wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
                            downTimer.reset();
                            downTimer.start();
                        },
                        () -> {
                            wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
                            if (downTimer.hasElapsed(0.3)) {
                                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                            }
                        },
                        (interrupted) -> {},
                        () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0)
        );

        super.addCommands(mechanismUp, mechanismDown);
        super.addRequirements(wrist, elevator, grabber);
    }

}
