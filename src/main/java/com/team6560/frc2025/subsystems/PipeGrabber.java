package com.team6560.frc2025.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PipeGrabber extends SubsystemBase {
    
    private SparkFlex grabberMotor;

    private static final int GRABBER_MOTOR_ID = 17;

    private static final double INTAKE_SPEED = 0.5;
    private static final double OUTTAKE_SPEED = -0.3;

    private static final double OUTTAKE_SPEED_L1 = -0.2;

    public PipeGrabber() {
        this.grabberMotor = new SparkFlex(GRABBER_MOTOR_ID, MotorType.kBrushless);
        ntDispTab("Grabber")
            .add("Grabber Duty Cycle", this::getDutyCycle);
    
    }

    public void runIntake(){
        
        grabberMotor.set(INTAKE_SPEED);
        // return;
        // if (!hasGamePiece()){
        //     grabberMotor.set(0.5);
        //     // grabberMotor.set(INTAKE_SPEED);
        // } else {
        //     grabberMotor.set(0.0);
        // }
    }

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
        // return this.grabberMotor.getReverseLimitSwitch().isPressed();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             ().isPressed();
        // with caleb inversion
        return false;
    }
    
    public double getOutputCurrent() {
        return grabberMotor.getOutputCurrent();
    }
}
