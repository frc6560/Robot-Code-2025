package com.team6560.frc2025.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;

/** Moves Wrist and Elevator to pickup, picks up piece */
public class StationIntake extends Command {

    private final Wrist wrist;
    private final Elevator elevator;
    private final PipeGrabber grabber;
    private final Timer timer = new Timer();
    private final double armDuration = 0.7; 
    private final double netDuration = 1.8;

    public StationIntake(Wrist wrist, Elevator elevator, PipeGrabber grabber) {
        this.wrist = wrist;
        this.elevator = elevator;
        this.grabber = grabber;
        addRequirements(wrist, elevator, grabber);
    }

    @Override
    public void initialize() {
        wrist.setMotorPosition(WristConstants.WristStates.STOW);
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if(timer.hasElapsed(armDuration)) {
            grabber.runIntakeMaxSpeed();
        }
        else{
            wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
        }
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stopMotor();
        elevator.stopMotors();
        grabber.stop();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(netDuration);
    }
}
