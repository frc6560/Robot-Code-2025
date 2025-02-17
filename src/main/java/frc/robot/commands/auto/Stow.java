package frc.robot.commands.auto;

import frc.robot.subsystems.Wrist;
import frc.robot.Constants.WristConstants;

import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class Stow extends Command{
    private Wrist wrist;
    private Elevator elevator;

    public Stow(Wrist wrist, Elevator elevator){
        this.wrist = wrist;
        this.elevator = elevator;

        addRequirements(wrist);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        wrist.setMotorPosition(WristConstants.WristStates.STOW);
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
    }

    @Override
    public void end(boolean interrupted){
        wrist.stopMotor();
        elevator.stopMotors();
    }

    @Override
    public boolean isFinished(){
        final double tolerance = 0.3;
        return (Math.abs(wrist.getWristAngle() - WristConstants.WristStates.STOW) < tolerance) && (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < tolerance);
    }
}
