package com.team6560.frc2025.commands.helpers;

import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.Wrist.State;
import com.team6560.frc2025.Constants.WristConstants;
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
      if(targetState == State.STOW){
        wrist.setMotorPosition(WristConstants.STOW_ANGLE);
      } else if (targetState == State.PICKUP){
        wrist.setMotorPosition(WristConstants.INTAKE_ANGLE);
      } else if (targetState == State.L2){
        wrist.setMotorPosition(WristConstants.L2_ANGLE);
      } else{
        wrist.setMotorPosition(WristConstants.L4_ANGLE);
      }
    }

    @Override
    public void execute(){
        wrist.handleState();
    }

    @Override
    public void end(boolean isFinished){
        wrist.stopMotor();
    }

    @Override
    public boolean isFinished(){
        return wrist.getState() == targetState;
    }
    
}
