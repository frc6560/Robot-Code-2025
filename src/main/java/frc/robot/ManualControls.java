package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class ManualControls {

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
    public ManualControls(XboxController firstXbox, XboxController secondXbox) {
        this.secondXbox = secondXbox;
        this.firstXbox = firstXbox;
        
    }

    public boolean getClimbUp() {
      // retun secondXbox.get
        return secondXbox.getRightBumperButton(); // Returns true while held
    }

    public boolean getClimbDown() {
        return secondXbox.getLeftBumperButton(); // Returns true while held
    }

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

    public boolean goToStow(){
        return secondXbox.getBackButton(); // TODO change to correct button
    }

    public boolean goToPickup(){
        return secondXbox.getRightTriggerAxis() > 0.25;
    }

    public boolean goToAlgae(){
        return secondXbox.getStartButton(); // TODO change to correct button
    }

    public boolean runIntake(){
      return secondXbox.getLeftTriggerAxis() > 0.25;
    }

    public boolean runOuttake() {
      return firstXbox.getRightBumperButton();
    }

    public boolean goToPlacePos() {
      return secondXbox.getLeftBumperButton();
    }

    public double testWrist(){
      return deadband(secondXbox.getRightX(), 0.1);
    }
    public double testEle(){
      return secondXbox.getLeftX();
    }
    public boolean resetWrist(){
      return secondXbox.getRightBumperButton();
    }
}
