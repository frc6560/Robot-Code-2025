package com.team6560.frc2025.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2025.subsystems.PipeGrabber;

import edu.wpi.first.wpilibj.Timer;

/** Moves Wrist and Elevator to pickup, picks up piece */
public class StationIntake extends Command {
    private final PipeGrabber grabber;
    private final Timer timer = new Timer();
    private final double netDuration = 1; 

    public StationIntake(PipeGrabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        // wrist.setMotorPosition(WristConstants.WristStates.STOW);
        // elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        grabber.runIntakeMaxSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(netDuration);
    }
}
