package com.team6560.frc2025.commands.auto;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;


public class MechanismDown extends Command{
    private final Wrist wrist;
    private final Elevator elevator;
    private final Timer timer = new Timer();

    public MechanismDown(Elevator elevator, Wrist wrist){
        this.wrist = wrist;
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        wrist.setMotorPosition(WristConstants.WristStates.PICKUP);
        if(timer.hasElapsed(0.1)){
            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
        }
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopMotors();
        wrist.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0;
    }
}
