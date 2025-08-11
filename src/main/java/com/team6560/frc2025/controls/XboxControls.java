package com.team6560.frc2025.controls;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControls {

    // private final XboxController secondXbox;
    private final XboxController secondXbox;
    private final XboxController firstXbox;
    
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }

      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.01);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }
      
    public XboxControls(XboxController firstXbox, XboxController secondXbox) {
        this.secondXbox = secondXbox;
        this.firstXbox = firstXbox;
        
    }

    // slow down

    public boolean slowDown() {
      return (firstXbox.getLeftTriggerAxis() > 0.25);
    }

    // climb

    public boolean getClimbDown() {
      return secondXbox.getRightY() > 0.7; 
        // return secondXbox.getLeftStickButton(); 
    }

    public boolean getClimbUp() {
      return secondXbox.getRightY() < -0.7;
        // return secondXbox.getRightStickButton(); 
    }

    // elevator

    public boolean goToL1(){
        return secondXbox.getAButton();
    }

    public boolean goToL2(){
        return secondXbox.getXButton();
    }

    public boolean goToL3(){
        return secondXbox.getBButton();
    }

    public boolean goToL4(){
        return secondXbox.getYButton();
    }

    public boolean shiftedControls(){
      return secondXbox.getRightBumperButton();
    }

    // wrist

    public boolean goToStow(){
        return secondXbox.getBackButton(); // TODO change to correct button
    }

    public boolean goToPickup(){
        return secondXbox.getRightTriggerAxis() > 0.25;
    }

    // pipe and ball grabber 

    // shifted for ball
    public boolean runGrabberIntake(){
      return secondXbox.getLeftTriggerAxis() > 0.25;
    }

    public boolean runGrabberOuttake() {
      return firstXbox.getRightBumper(); // change if inconvenient
    }
    public boolean runGrabberOuttakeL1() {
      return firstXbox.getLeftBumperButton();
    }

    public boolean goToPlacePos() {
      return secondXbox.getLeftBumperButton();
    }

    // tests 

    // public double testWrist(){
    //   return deadband(secondXbox.getRightX(), 0.1);
    // }
    // public double testEle(){
    //   return secondXbox.getLeftX();
    // }
    // public boolean resetWrist(){
    //   return secondXbox.getRightBumperButton();
    // }
}
