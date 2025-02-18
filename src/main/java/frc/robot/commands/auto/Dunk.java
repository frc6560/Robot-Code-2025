package frc.robot.commands.auto;

import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;
import frc.robot.Constants.WristConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class Dunk extends Command{
    private Wrist wrist;
    private int outtake = -1;

    public Dunk(Wrist wrist){
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        wrist.setMotorPosition(WristConstants.WristStates.L2 + outtake * WristConstants.WristStates.L2Offset);
    }

    @Override
    public void end(boolean interrupted){
        wrist.stopMotor();
    }

    @Override
    public boolean isFinished(){
        final double tolerance = 0.1;
        return Math.abs(wrist.getWristAngle() - WristConstants.WristStates.L2 - outtake * WristConstants.WristStates.L2Offset) < tolerance;
    }
}