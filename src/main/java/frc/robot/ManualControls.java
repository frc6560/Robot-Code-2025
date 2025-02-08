package frc.robot;

import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.XboxController;

public class ManualControls {

    // private final XboxController secondXbox;
    private final XboxController secondXbox;
    
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
    public ManualControls(XboxController secondXbox) {
        this.secondXbox = secondXbox;
        
    }

    public boolean grabberOuttake() {
        return secondXbox.getBButton(); // Returns true while held
    }

    public boolean grabberIntake() {
        return secondXbox.getAButton(); // Returns true while held
    }
    public boolean getRunClimb() {
      // retun secondXbox.get
        return secondXbox.getRightBumperButton(); // Returns true while held
    }
    public boolean getDownClimb() {
        return secondXbox.getLeftBumperButton(); // Returns true while held
    }
    public double getClimbSpeed() {
        return - modifyAxis(secondXbox.getLeftY());
    }

    public boolean getRunScoreL1(){
        return secondXbox.getAButton();
    }

    public boolean getRunScoreL2(){
        return secondXbox.getBButton();
    }

    public boolean getRunScoreL3(){
        return secondXbox.getXButton();
    }

    public boolean getRunScoreL4(){
        return secondXbox.getYButton();
    }

    public boolean getRunScoreStow(){
        return secondXbox.getBackButton(); // TODO change to correct button
    }

    public boolean getRunScorePickup(){
        return secondXbox.getStartButton();
    }

    public boolean getRunScoreBall(){
        return secondXbox.getBackButton(); // TODO change to correct button
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
