package com.team6560.frc2025.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;

public class ScoringL4DownOnly extends SequentialCommandGroup{

    public ScoringL4DownOnly(Wrist wrist, Elevator elevator, PipeGrabber grabber) {
        double wristAngleL4 = 6.56;
        Timer downTimer = new Timer();

        final Command mechanismDown = new FunctionalCommand(
                        () -> {
                            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                            wrist.setMotorPosition(WristConstants.WristStates.STOW);
                            downTimer.reset();
                            downTimer.start();
                        },
                        () -> {
                            wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
                            if (downTimer.hasElapsed(0.7)) {
                                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                            }
                        },
                        (interrupted) -> {},
                        () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0)
        );

        super.addCommands(mechanismDown);
        super.addRequirements(wrist, elevator, grabber);
    }

}
