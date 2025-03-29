package com.team6560.frc2025.commands;

import com.team6560.frc2025.Constants;
import com.team6560.frc2025.ManualControls;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.Elevator.State;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {

    private final Elevator elevator;
    private final ManualControls controls;

    private State targetState;

    private double targetrotelev = -1;

    public ElevatorCommand(Elevator elevator, ManualControls controls) {
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

        if (controls.shiftedControls()) {
            if (controls.goToL2()) {
                targetState = State.S_L2;
            } else if (controls.goToL3()) {
                targetState = State.S_L3;
            } else if (controls.goToL4()) {
                targetState = State.S_L4;
            } else if (controls.goToStow()) {
                targetState = State.S_STOW;
            }
        } else {
            if (controls.goToL1() || controls.goToPickup() || controls.goToStow()) {
                targetState = State.STOW;
            } else if (controls.goToL2()) {
                targetState = State.L2;
            } else if (controls.goToL3()) {
                targetState = State.L3;
            } else if (controls.goToL4()) {
                targetState = State.L4;
            }
        }

        if (targetState == State.STOW) {
            targetrotelev = ElevatorConstants.ElevatorStates.STOW;
        } else if (targetState == State.L2) {
            targetrotelev = ElevatorConstants.ElevatorStates.L2;
        } else if (targetState == State.L3) {
            targetrotelev = ElevatorConstants.ElevatorStates.L3;
        } else if (targetState == State.L4) {
            targetrotelev = ElevatorConstants.ElevatorStates.L4;
        } else if (targetState == State.S_L2) {
            targetrotelev = ElevatorConstants.ElevatorStates.S_L2;
        } else if (targetState == State.S_L3) {
            targetrotelev = ElevatorConstants.ElevatorStates.S_L3;
        } else if (targetState == State.S_L4) {
            targetrotelev = ElevatorConstants.ElevatorStates.S_L4;
        } else if (targetState == State.S_STOW) {
            targetrotelev = ElevatorConstants.ElevatorStates.S_STOW;
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
