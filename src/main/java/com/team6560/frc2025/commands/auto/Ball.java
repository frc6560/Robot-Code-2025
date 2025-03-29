package com.team6560.frc2025.commands.auto;


import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.BallGrabber;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;

public class Ball extends Command {
    
    private final BallGrabber grabber;
    private final Wrist wrist;
    private final Elevator elevator;
    private final Timer timer = new Timer();

    private final double elevatorUpTime = 0.3;
    private final double wristDownTime = 1.0;
    private final double ejectTime = 1.3;
    private final double wristUpTime = 1.7;
    private final double elevatorDownTime = 2.2;
    private final double totalTime = 2.2;

    public Ball(Wrist wrist, Elevator elevator, BallGrabber grabber) {

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

        if (currentTime < elevatorUpTime) {
            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.S_L3);
        } else if (currentTime < wristDownTime) {
            wrist.setMotorPosition(WristConstants.WristStates.S_L2);
        } else if (currentTime < wristUpTime) {
            grabber.runIntake();
        } else if (currentTime < elevatorDownTime) {
            grabber.stop();
            wrist.setMotorPosition(WristConstants.WristStates.STOW);
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
