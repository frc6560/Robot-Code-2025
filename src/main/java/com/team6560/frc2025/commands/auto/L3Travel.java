package com.team6560.frc2025.commands.auto;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;

public class L3Travel extends Command {
    private final Wrist wrist;
    private final Elevator elevator;

    public L3Travel(Wrist wrist, Elevator elevator){
        this.wrist = wrist;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
    }

    @Override
    public void execute(){
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.L4);
        wrist.setMotorPosition(WristConstants.WristStates.STOW);
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopMotors();
        wrist.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.L3) < 1.5;
    }
}
