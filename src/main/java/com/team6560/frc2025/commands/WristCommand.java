package com.team6560.frc2025.commands;

import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.controls.XboxControls;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.Wrist.State;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;


/** A helper command that controls the Wrist subsystem */
public class WristCommand extends Command {

    final Wrist wrist;

    State targetState = State.STOW;
    final XboxControls controls;
    DigitalInput LimitSwitch = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT_ID);

    public WristCommand(Wrist wrist, XboxControls controls){
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

      if(controls.goToPickup()) {

          targetState = State.PICKUP;

        } 
      if (targetState == State.PICKUP){

        wrist.setMotorPosition(WristConstants.WristStates.PICKUP);

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
