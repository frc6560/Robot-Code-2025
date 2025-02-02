package com.team6560.frc2025.commands.helpers;

import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final int targetState;

    public ElevatorCommand(Elevator elevator, int targetState){
        this.elevator = elevator;
        this.targetState = targetState;
    }

    @Override
    public void initialize() {
        elevator.setelevpos(targetState);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getElevatorHeight() - targetState) < 5;
    }
}
