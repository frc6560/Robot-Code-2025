package frc.robot.commands.auto;

import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployElevatorL4 extends Command{
    private Elevator elevator;

    public DeployElevatorL4(Elevator elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize(){
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
    }

    @Override
    public void execute(){
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.L4);
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopMotors();
    }

    @Override
    public boolean isFinished(){
        final double tolerance = 0.5;
        return Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.L4) < tolerance;
    }

}
