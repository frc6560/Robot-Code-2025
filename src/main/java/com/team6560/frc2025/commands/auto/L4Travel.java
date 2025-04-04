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

    private final double wristStateL4 = 40.0;
    private final double elevStateL4 = 17.65;

    public L4Travel(Elevator elevator, Wrist wrist, double time){
        this.wrist = wrist;
        this.elevator = elevator;
        this.slewratelimiterelev = new SlewRateLimiter((elevStateL4 - ElevatorConstants.ElevatorStates.STOW) / time);
        this.slewratelimiterwrist = new SlewRateLimiter((WristConstants.WristStates.PICKUP - wristStateL4) / time);
    }

    @Override
    public void initialize() {
        slewratelimiterelev.reset(ElevatorConstants.ElevatorStates.STOW);
        slewratelimiterwrist.reset(WristConstants.WristStates.PICKUP);
    }

    @Override
    public void execute(){
        elevator.setElevatorPosition(slewratelimiterelev.calculate(elevStateL4));
        wrist.setMotorPosition(slewratelimiterwrist.calculate(wristStateL4));
    }

    @Override
    public void end(boolean interrupted){
        elevator.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getElevatorHeight() - elevStateL4) < 1.5;
    }
}
