package frc.robot.subsystems;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    
    private SparkMax grabberMotor;

    private static final int GRABBER_MOTOR_ID = 17;

    private static final double INTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED = 0.3;
    private static final double CURRENT_LIMIT = 40.0; // Stop the motor if current exceeds 40A
    private boolean hasGamePiece = false; // Track whether the grabber is holding something

    public Grabber() {
        this.grabberMotor = new SparkMax(GRABBER_MOTOR_ID, MotorType.kBrushless);

        ntDispTab("Grabber")
            .add("Grabber Duty Cycle", this::getDutyCycle);
    
    }

    public void runIntake(){
        if (!hasGamePiece()){
            grabberMotor.set(INTAKE_SPEED);
        } else {
            grabberMotor.set(0.0);
        }
    }

    public void runOuttake(){
        grabberMotor.set(OUTTAKE_SPEED);
    }

    public void stop() {
        grabberMotor.set(0);
    }

    public double getDutyCycle() {
        return grabberMotor.get();
    }

    public boolean hasGamePiece() {
        return false; // need to write stuff using a sensor
    }
    public double getOutputCurrent() {
        return grabberMotor.getOutputCurrent();
    }
}
