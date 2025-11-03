package com.team6560.frc2025.controls;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControls {
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

    // --- CLIMB ---

    public boolean getClimbDown() {
      return firstXbox.getRightBumperButton(); 
    }

    public boolean getClimbUp() {
      return firstXbox.getLeftBumperButton();
    }

    // // --- SUPERSTRUCTURE ---
    public boolean goToPickup(){
        return firstXbox.getRightTriggerAxis() > 0.25;
    }

    // --- END EFFECTORS ---

    public boolean runGrabberOuttake() {
      return firstXbox.getLeftTriggerAxis() > 0.25;
    }
}
