package frc.robot.commands;

import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Grabber;
import frc.robot.ManualControls;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;


/** A helper command that controls the Wrist subsystem */
public class WristCommand extends Command{
    final Wrist wrist;
    State targetState = State.STOW;
    final ManualControls controls;
    DigitalInput LimitSwitch = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT_ID);


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
      int outtake = 0;

      if(controls.goToL1()){
        targetState = State.L1;

      } else if(controls.goToL2() || controls.goToL3()){
        targetState = State.L2;

      } else if(controls.goToL4()){
        targetState = State.L4;

      } else if(controls.goToStow()){
        targetState = State.STOW;

      } else if(controls.goToPickup()) {
        targetState = State.PICKUP;

      } else if (controls.runOuttake()) {
        outtake = 1;

      } 


      // if (controls.resetWrist() || LimitSwitch.get()) {
      //   // System.out.println("Restesting");
        
      //   wrist.setEncoderPosition(0.0);
      // }

      
        

      
      if (Math.abs(controls.testWrist()) > 0.15){
        wrist.testMotor(controls.testWrist());

      } else if(targetState == State.STOW){
        wrist.setMotorPosition(WristConstants.WristStates.STOW + outtake * WristConstants.WristStates.StowOffset);

      } else if (targetState == State.PICKUP){
        wrist.setMotorPosition(WristConstants.WristStates.PICKUP + outtake * WristConstants.WristStates.PickupOffset);

      } else if (targetState == State.L1){
        wrist.setMotorPosition(WristConstants.WristStates.L1 + outtake * WristConstants.WristStates.L1Offset);

      } else if (targetState == State.L2){
        wrist.setMotorPosition(WristConstants.WristStates.L2 + outtake * WristConstants.WristStates.L2Offset);

      } else if (targetState == State.L4){
        wrist.setMotorPosition(WristConstants.WristStates.L4 + outtake * WristConstants.WristStates.L4Offset);
      
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
