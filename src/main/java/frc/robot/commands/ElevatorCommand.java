package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;


public class ElevatorCommand extends Command{
    
    private Elevator elevator;

    public ElevatorCommand(Elevator elevator) {
        this.elevator = elevator;
    }

    public void initialize() {
        elevator.stopMotors();
    }

    public void periodic() {

        if (elevator.bottomLimitSwitchDown() == true) {
            elevator.resetEncoderPos(0);
        }

        else if (elevator.topLimitSwitchDown() == true) {
            elevator.resetEncoderPos(15);
        }

    }

    public void execute() {
        
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() {
        return false;
    }

}
