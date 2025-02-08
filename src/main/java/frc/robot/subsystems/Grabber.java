package frc.robot.subsystems;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    
    private SparkMax grabberMotor;
    private final SlewRateLimiter rateLimiter;

    private static final int GRABBER_MOTOR_ID = 17;

    private static final double INTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED = 0.3;
    private static final double CURRENT_LIMIT = 70.0; // Stop the motor if current exceeds 40A
    private boolean hasGamePiece = false; // Track whether the grabber is holding something

    public Grabber() {

        this.grabberMotor = new SparkMax(GRABBER_MOTOR_ID, MotorType.kBrushless);
        rateLimiter = new SlewRateLimiter(1.0 / 0.5); // TODO: get rid of this limiter, you dont need a limiter for a simple roller, adds unnecessary complication

        ntDispTab("Grabber")
            .add("Grabber Duty Cycle", this::getDutyCycle);
    
    }

public void intake() {
 if (!hasGamePiece) {
                // Continue intaking if no piece is detected
                grabberMotor.set(rateLimiter.calculate(INTAKE_SPEED));
                System.out.println("Intaking - Current: " + getOutputCurrent() + "A");
                
            if (getOutputCurrent() >= CURRENT_LIMIT) {
                // High current means we're grabbing something
                hasGamePiece = true;
                grabberMotor.set(0);
            }
        }
else {
            // If we already have a game piece, stop intaking
            stop();
            System.out.println("✅ Game piece detected, stopping intake.");
            return;
}
        
        
            
}

public void outtake() {
    grabberMotor.set(OUTTAKE_SPEED);
    hasGamePiece = false;
}

    public void stop() {
        grabberMotor.set(0);
    }

    public double getDutyCycle() {
        return grabberMotor.get();
    }
    public boolean hasGamePiece() {
        return hasGamePiece;
    }
    public double getOutputCurrent() {
        return grabberMotor.getOutputCurrent();
    }
}
