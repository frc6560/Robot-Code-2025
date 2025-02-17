package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.State;
import frc.robot.ManualControls;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCommand extends Command {
    private final Elevator elevator;
    private final ManualControls controls;
    
    private State targetState;
    

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
        if(controls.goToL1()
            || controls.goToPickup()
            || controls.goToStow()){
            targetState = State.STOW;

        }
        else if(controls.goToL2()){
            targetState = State.L2;

        } else if(controls.goToL3()){
            targetState = State.L3;
            
        } else if(controls.goToL4()){
            targetState = State.L4;

        } else if(controls.goToAlgae()){
            targetState = State.BALL;
        } 
        
        double targetrotelev = 0;

        if (targetState == State.STOW) {
            targetrotelev = ElevatorConstants.ElevatorStates.STOW;
        }
        else if (targetState == State.L2) {
            targetrotelev = ElevatorConstants.ElevatorStates.L2;
        }
        else if (targetState == State.L3) {
            targetrotelev = ElevatorConstants.ElevatorStates.L3;
        }
        else if (targetState == State.L4) {
            targetrotelev = ElevatorConstants.ElevatorStates.L4;
        }
        else if (targetState == State.BALL) {
            targetrotelev = ElevatorConstants.ElevatorStates.BALL;
        }

        elevator.setElevatorPosition(targetrotelev);
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