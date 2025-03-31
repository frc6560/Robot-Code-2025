package com.team6560.frc2025.commands.auto;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class L4Travel extends Command {
    private final Wrist wrist;
    private final Elevator elevator;
    private final SlewRateLimiter slewratelimiterelev;
    private final SlewRateLimiter slewratelimiterwrist;

    public L4Travel(Elevator elevator, Wrist wrist){
        this.wrist = wrist;
        this.elevator = elevator;
        this.slewratelimiterelev = new SlewRateLimiter((ElevatorConstants.ElevatorStates.L4-ElevatorConstants.ElevatorStates.STOW)/1.94);
        this.slewratelimiterwrist = new SlewRateLimiter((WristConstants.WristStates.PICKUP - WristConstants.WristStates.L4) / 1.94);
    }

    @Override
    public void initialize() {
        slewratelimiterelev.reset(ElevatorConstants.ElevatorStates.STOW);
        slewratelimiterwrist.reset(WristConstants.WristStates.PICKUP);
    }

    @Override
    public void execute(){
        elevator.setElevatorPosition(slewratelimiterelev.calculate(ElevatorConstants.ElevatorStates.L4));
        wrist.setMotorPosition(slewratelimiterwrist.calculate(WristConstants.WristStates.L4));
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.L4) < 1.5;
    }
}
