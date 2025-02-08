package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ManualControls {

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
        return secondXbox.getRightBumper(); // Returns true while held
    }
    public boolean getDownClimb() {
        return secondXbox.getLeftBumper(); // Returns true while held
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
}
