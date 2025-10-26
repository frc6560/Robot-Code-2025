package com.team6560.frc2025.commands;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.controls.XboxControls;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Elevator.State;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {

    private final Elevator elevator;
    private final XboxControls controls;

    private State targetState;

    private double targetrotelev = -1;

    public ElevatorCommand(Elevator elevator, XboxControls controls) {
        this.elevator = elevator;
        this.controls = controls;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
    }

    @Override
    public void execute() {
            if (controls.goToPickup()) {
                targetState = State.STOW;
            } 

        if (targetState == State.STOW) {
            targetrotelev = ElevatorConstants.ElevatorStates.STOW;
        } 

        elevator.setElevatorPosition(targetrotelev);
    }

    public boolean isAtTargetPos() {
        return Math.abs(elevator.getElevatorHeight() - targetrotelev) < 1.0;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotors();
    }
}
