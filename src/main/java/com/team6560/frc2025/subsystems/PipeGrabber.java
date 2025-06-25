package com.team6560.frc2025.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// This is a climb subsystem.
public class PipeGrabber extends SubsystemBase {
    // Initializes all the motors you'll need, along with some constants. 
    private SparkFlex grabberMotor; // This is the motor

    private static final int GRABBER_MOTOR_ID = 17;
    private static final double INTAKE_SPEED = 0.5;
    private static final double OUTTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED_L1 = -0.2;

    // This is your constructor. Creates a new PipeGrabber.
    public PipeGrabber() {
        this.grabberMotor = new SparkFlex(GRABBER_MOTOR_ID, MotorType.kBrushless);
        // This is for telemetry.
        ntDispTab("Grabber")
            .add("Grabber Duty Cycle", this::getDutyCycle);
    
    }

    // These are the methods that make the grabber run.
    public void runIntake(){
        // This sets the motor's velocity to INTAKE_SPEED
        grabberMotor.set(INTAKE_SPEED);
    }

    // These follow the same structure as above!
    public void runIntakeMaxSpeed() {
        grabberMotor.set(1.0);
    }

    public void runGrabberOuttake(){
        grabberMotor.set(OUTTAKE_SPEED);
        // System.out.println("Worked");
    }

    public void runGrabberOuttakeL1() {
        grabberMotor.set(OUTTAKE_SPEED_L1);
    }

    public void runGrabberOuttakeMaxSpeed() {
        grabberMotor.set(-1.0);
    }

    public void stop() {
        grabberMotor.set(0);
    }

    public double getDutyCycle() {
        return grabberMotor.get();
    }

    public boolean hasGamePiece() {
        return false;
    }
    
    public double getOutputCurrent() {
        return grabberMotor.getOutputCurrent();
    }
}
