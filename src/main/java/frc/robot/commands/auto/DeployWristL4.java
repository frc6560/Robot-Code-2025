package frc.robot.commands.auto;

// import frc.robot.subsystems.Wrist.State;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants.WristConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class DeployWristL4 extends Command{
    private Wrist wrist;

    public DeployWristL4(Wrist wrist){
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void initialize(){
        wrist.setMotorPosition(WristConstants.WristStates.STOW);
    }

    @Override
    public void execute(){
        wrist.setMotorPosition(WristConstants.WristStates.L4);
    }

    @Override
    public void end(boolean interrupted){
        wrist.stopMotor();
    }

    @Override
    public boolean isFinished(){
        final double tolerance = 0.5; // adjust later
        return Math.abs(wrist.getWristAngle() - WristConstants.WristStates.L4) < tolerance;
    }
}
