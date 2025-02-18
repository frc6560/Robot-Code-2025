package frc.robot.subsystems;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PipeGrabber extends SubsystemBase {
    
    private SparkMax grabberMotor;

    private static final int GRABBER_MOTOR_ID = 17;

    private static final double INTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED = 0.3;

    public PipeGrabber() {
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

    public void runGrabberOuttake(){
        grabberMotor.set(OUTTAKE_SPEED);
    }

    public void stop() {
        grabberMotor.set(0);
    }

    public double getDutyCycle() {
        return grabberMotor.get();
    }

    public boolean hasGamePiece() {
        return this.grabberMotor.getReverseLimitSwitch().isPressed();
        // with caleb inversion
    }
    public double getOutputCurrent() {
        return grabberMotor.getOutputCurrent();
    }
}
