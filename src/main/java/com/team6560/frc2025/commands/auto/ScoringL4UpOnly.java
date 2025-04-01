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

public class ScoringL4UpOnly extends SequentialCommandGroup{

    public ScoringL4UpOnly(Wrist wrist, Elevator elevator, PipeGrabber grabber) {
        double wristAngleL4 = 6.56;
        Timer ejectTimer = new Timer();
        Timer wristTimer = new Timer();

        final Command mechanismUp = new FunctionalCommand(
                    () -> {
                    },
                    () -> {
                        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.L4);
                        wrist.setMotorPosition(wristAngleL4);
                    },
                    (interrupted) -> {},
                    () -> Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.L4) < 1.0 
                          // && Math.abs(wrist.getWristAngle() - wristAngleL4) < 40.0
                    );

        final Command ejectPiece = new FunctionalCommand(
                        () -> {
                            ejectTimer.reset();
                            ejectTimer.start();
                        },
                        () -> {
                            if (ejectTimer.hasElapsed(0.2)) { // jank fix but wrist pos check doesn't work :(
                                grabber.runGrabberOuttakeMaxSpeed();
                            }
                            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.L4);
                            wrist.setMotorPosition(wristAngleL4);
                        },
                        (interrupted) -> grabber.stop(), 
                        () -> ejectTimer.hasElapsed(0.4));

        final Command wristToStow = new FunctionalCommand(
                        () -> {
                            wristTimer.reset();
                            wristTimer.start();
                        },
                        () -> {
                            wrist.setMotorPosition(WristConstants.WristStates.STOW);
                        },
                        (interrupted) -> grabber.stop(), 
                        () -> ejectTimer.hasElapsed(0.4));


        super.addCommands(mechanismUp, ejectPiece, wristToStow);
        super.addRequirements(wrist, elevator, grabber);
    }

}
