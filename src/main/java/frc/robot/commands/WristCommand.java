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
    final Grabber grabber;
    State targetState = State.STOW;
    final ManualControls controls;
    DigitalInput LimitSwitch = new DigitalInput(WristConstants.LIMIT_SWITCH_PORT_ID);


    public WristCommand(Wrist wrist, ManualControls controls, Grabber grabber){
      this.wrist = wrist;
      this.controls = controls;
      this.grabber = grabber;

      addRequirements(wrist);
    }

    @Override
    public void initialize(){
      wrist.setMotorPosition(WristConstants.WristStates.STOW);
    }

    @Override
    public void execute(){

      if(controls.goToL1()){
        targetState = State.L1;

      } else if(controls.goToL2() || controls.goToL3()){
        targetState = State.L2;

      } else if(controls.goToL4()){
        targetState = State.L4;

      } else if(controls.goToStow()){
        targetState = State.STOW;

      } else if(controls.goToPickup()){
        targetState = State.PICKUP;

      } else if (controls.runOuttake()) {
        //move wrist depending on current state
        if (targetState == State.L2) {
          targetState = State.GrabberL2;
        } else if (targetState == State.L4) {
          targetState = State.GrabberL4;
        } else {}
        grabber.runOuttake();
      }
      // wrist.stopMotor();


      if (controls.resetWrist() || LimitSwitch.get()) {
        // System.out.println("Restesting");
        
        wrist.setEncoderPosition(0.0);
      }

      
      if (Math.abs(controls.testWrist()) > 0.15){
        wrist.testMotor(controls.testWrist());

      } else if(targetState == State.STOW){
        wrist.setMotorPosition(WristConstants.WristStates.STOW);

      } else if (targetState == State.PICKUP){
        wrist.setMotorPosition(WristConstants.WristStates.PICKUP);

      } else if (targetState == State.L1){
        wrist.setMotorPosition(WristConstants.WristStates.L1);

      } else if (targetState == State.L2){
        wrist.setMotorPosition(WristConstants.WristStates.L2);

      } else if (targetState == State.L4){
        wrist.setMotorPosition(WristConstants.WristStates.L4);

      } else if (targetState == State.GrabberL2){
        wrist.setMotorPosition(WristConstants.WristStates.L2 - 20);

      } else if (targetState == State.GrabberL4){
        wrist.setMotorPosition(WristConstants.WristStates.L4 - 20);
      
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
