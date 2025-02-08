package frc.robot.commands;

import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;
import frc.robot.Constants.WristConstants;
import frc.robot.ManualControls;
import edu.wpi.first.wpilibj2.command.Command;


/** A helper command that controls the Wrist subsystem */
public class WristCommand extends Command{
    final Wrist wrist;
    State targetState = State.STOW;
    final ManualControls controls;

    public WristCommand(Wrist wrist, ManualControls controls){
      this.wrist = wrist;
      this.controls = controls;

      addRequirements(wrist);
    }

    @Override
    public void initialize(){
      wrist.setMotorPosition(WristConstants.WristStates.STOW);
    }

    @Override
    public void execute(){

      if(controls.getRunScoreL2() || controls.getRunScoreL3()){
        targetState = State.L2;

      } else if(controls.getRunScoreL4()){
        targetState = State.L4;

      } else if(controls.getRunScoreStow()){
        targetState = State.STOW;

      } else if(controls.getRunScorePickup()){
        targetState = State.PICKUP;
      }

      

      if(targetState == State.STOW){
        wrist.setMotorPosition(WristConstants.WristStates.STOW);

      } else if (targetState == State.PICKUP){
        wrist.setMotorPosition(WristConstants.WristStates.PICKUP);

      } else if (targetState == State.L1){
        wrist.setMotorPosition(WristConstants.WristStates.L1);

      } else if (targetState == State.L2){
        wrist.setMotorPosition(WristConstants.WristStates.L2);

      } else if (targetState == State.L4){
        wrist.setMotorPosition(WristConstants.WristStates.L4);
      }
    }

    @Override
    public void end(boolean interrupted){
        wrist.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
}
