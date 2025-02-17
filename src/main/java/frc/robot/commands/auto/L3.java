package frc.robot.commands.auto;

import frc.robot.subsystems.Wrist;
import frc.robot.Constants.WristConstants;

import frc.robot.subsystems.Elevator;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class L3 extends Command{
    private Wrist wrist;
    private Elevator elevator;

    public L3(Wrist wrist, Elevator elevator){
        this.wrist = wrist;
        this.elevator = elevator;

        addRequirements(wrist);
    }

    @Override
    public void initialize(){
        wrist.setMotorPosition(WristConstants.WristStates.STOW);
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
    }

    @Override
    public void execute(){
        wrist.setMotorPosition(WristConstants.WristStates.L2);
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.L3);
    }

    @Override
    public void end(boolean interrupted){
        wrist.stopMotor();
        elevator.stopMotors();
    }

    @Override
    public boolean isFinished(){
        final double tolerance = 0.3;
        return (Math.abs(wrist.getWristAngle() - WristConstants.WristStates.L2) < tolerance) 
        && (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.L3) < tolerance);
    }
}
