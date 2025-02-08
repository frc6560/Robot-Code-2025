package frc.robot.commands.helpers;

import frc.robot.subsystems.Elevator;
import frc.robot.frc2025.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final int targetState;

    private final int[] elevatorPositions = {0, 3, 6, 9};

    public ElevatorCommand(Elevator elevator, int targetState){
        this.elevator = elevator;
        this.targetState = targetState;

        addRequirements(elevator);
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
        return Math.abs(elevator.getElevatorHeight() - elevatorPositions[targetState-1] * ElevatorConstants.ELEV_GEAR_RATIO) < 5;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopMotors();
    }
}