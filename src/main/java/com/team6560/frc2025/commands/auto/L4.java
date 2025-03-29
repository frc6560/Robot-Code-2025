package com.team6560.frc2025.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;

// complete L4 sequence

// see if this can be implemented w/o timers (not critical but good overall)

public class L4 extends Command {
    
    private final PipeGrabber grabber;
    private final Wrist wrist;
    private final Elevator elevator;
    private final Timer timer = new Timer();

    private final double elevatorUpTime = 0.4;
    private final double wristDownTime = 1.3;
    private final double ejectTime = 1.35;
    private final double wristUpTime = 1.6;
    private final double elevatorDownTime = 1.7;
    private final double totalTime = 2.2;

    public L4(Wrist wrist, Elevator elevator, PipeGrabber grabber) {
        this.grabber = grabber;
        this.wrist = wrist;
        this.elevator = elevator;
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

        double currentTime = timer.get();


        double elevL4 = 18.8; // 17.4
        double wristAngleL4 = 6.56; // 0

        if (currentTime < elevatorUpTime) {
            elevator.setElevatorPosition(elevL4);
        } else if (currentTime < wristDownTime) {
            wrist.setMotorPosition(wristAngleL4);
        } else if (currentTime < ejectTime) {
            // wrist.setMotorPosition(wristAngleL4);
        } else if (currentTime < wristUpTime) {
            grabber.runGrabberOuttakeMaxSpeed();
        } else if (currentTime < elevatorDownTime) {
            grabber.stop();
            wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
        } else {
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
        return timer.hasElapsed(totalTime);
    }
}
