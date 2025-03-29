package com.team6560.frc2025.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
public class L3Travel extends Command {
    
    private final Wrist wrist;
    private final Elevator elevator;
    private final Timer timer = new Timer();

    private final SlewRateLimiter elevatorLimiter;
    private final SlewRateLimiter wristLimiter;

    private final double elevatorInitialHeight = ElevatorConstants.ElevatorStates.STOW;
    private final double elevatorFinalHeight = 7.0;

    private final double wristInitialPos = WristConstants.WristStates.PICKUP;
    private final double wristFinalPos = 70.0;

    private final double pathDuration;


    public L3Travel(Wrist wrist, Elevator elevator, double pathDuration) {
        this.wrist = wrist;
        this.elevator = elevator; 
        this.elevatorLimiter = new SlewRateLimiter((Math.abs(elevatorFinalHeight - elevatorInitialHeight) / pathDuration) + 2);
        this.wristLimiter = new SlewRateLimiter(((360-225)+70) / pathDuration); // avoid 360 deg thing
        this.pathDuration = pathDuration;
        addRequirements(wrist, elevator);
    }

    @Override
    public void initialize() {

        wrist.setMotorPosition(WristConstants.WristStates.STOW);
        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);

        elevatorLimiter.reset(elevator.getElevatorHeight());
        wristLimiter.reset(wrist.getWristPosition());

        timer.reset();
        timer.start();

    }

    @Override
    public void execute() {

        elevator.setElevatorPosition(elevatorLimiter.calculate(this.elevatorFinalHeight));
        wrist.setMotorPosition(wristLimiter.calculate(this.wristFinalPos));

    }

    @Override
    public void end(boolean interrupted) {

        wrist.stopMotor();
        elevator.stopMotors();
        timer.stop();

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(pathDuration);
    }
}
