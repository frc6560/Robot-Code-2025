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

      int outtake = 0;

      if(controls.shiftedControls()){

        if (controls.goToL2() || controls.goToL3()){

          targetState = State.S_L2;

        } else if(controls.goToL4()){

          targetState = State.S_L4;

        } else if (controls.goToStow()) {
          targetState = State.S_STOW;

        }

      } else { 

        if(controls.goToL1()){

          targetState = State.L1;
  
        } else if(controls.goToL2()){

          targetState = State.L2;

        } else if(controls.goToL3()){

          targetState = State.L3;

        }else if(controls.goToL4()){

          targetState = State.L4;

        } else if(controls.goToStow()){

          targetState = State.STOW;

        } else if(controls.goToPickup()) {

          targetState = State.PICKUP;

        } else if (controls.goToPlacePos()) {

          outtake = -1;

        } 

      }
    
      if(targetState == State.STOW){

        wrist.setMotorPosition(WristConstants.WristStates.STOW + outtake * WristConstants.WristStates.StowOffset);

      } else if (targetState == State.PICKUP){

        wrist.setMotorPosition(WristConstants.WristStates.PICKUP + outtake * WristConstants.WristStates.PickupOffset);

      } else if (targetState == State.L1){

        wrist.setMotorPosition(WristConstants.WristStates.L1 + outtake * WristConstants.WristStates.L1Offset);

      } else if (targetState == State.L2) {

        wrist.setMotorPosition(WristConstants.WristStates.L2 + outtake * WristConstants.WristStates.L2Offset);

      } else if (targetState == State.L3) {

        wrist.setMotorPosition(WristConstants.WristStates.L2 + outtake * WristConstants.WristStates.L2Offset);

      } else if (targetState == State.L4) {

        wrist.setMotorPosition(WristConstants.WristStates.L4 + outtake * WristConstants.WristStates.L4Offset);
      
      } else if (targetState == State.S_L2){

        wrist.setMotorPosition(WristConstants.WristStates.S_L2);

      } else if (targetState == State.S_L4){

        wrist.setMotorPosition(WristConstants.WristStates.S_L4);

      } else if (targetState == State.S_STOW) {

        wrist.setMotorPosition(WristConstants.WristStates.S_STOW);

      } else {}

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
