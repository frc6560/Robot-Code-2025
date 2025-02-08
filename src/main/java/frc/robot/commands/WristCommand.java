
package frc.robot.commands.helpers;

import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.Command;


/** A helper command that controls the Wrist subsystem */
public class WristCommand extends Command{
    final Wrist wrist;
    final State targetState;

    public WristCommand(Wrist wrist, State targetState){
      this.wrist = wrist;
      this.targetState = targetState;

      addRequirements(wrist);
    }

    @Override
    public void initialize(){
      wrist.setMotorPosition(WristConstants.STOW_ANGLE);
    }

    @Override
    public void execute(){

      if(controls.getRunScoreL2() || controls.getRunScoreL3()){
        targetState  = State.L2;

      } else if(controls.getRunScoreL4()){
        targetState = State.L4;

      } else if(controls.getRunScoreStow()){
        targetState = State.STOW;

      } else if(controls.getRunScorePickup()){
        targetState = State.PICKUP;
      }

      

      if(targetState == State.STOW){
        wrist.setMotorPosition(WristConstants.STOW_ANGLE);

      } else if (targetState == State.PICKUP){
        wrist.setMotorPosition(WristConstants.INTAKE_ANGLE);

      } else if (targetState == State.L2){
        wrist.setMotorPosition(WristConstants.L2_ANGLE);

      } else if (targetState == State.L4){
        wrist.setMotorPosition(WristConstants.L4_ANGLE);
      }
    }

    @Override
    public void end(boolean interrupted){
        wrist.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return wrist.getState() == targetState;
    }
    
}
