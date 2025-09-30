package com.team6560.frc2025.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallGrabber extends SubsystemBase {
    
    private SparkMax grabberMotor;

    private static final int GRABBER_MOTOR_ID = 25; 

    private static final double INTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED = 0.7;

    // max current where the ball isn't wedged in
    private static final double MAX_CURRENT_RUNNING = 30;
    public BallGrabber() {
        this.grabberMotor = new SparkMax(GRABBER_MOTOR_ID, MotorType.kBrushless);
        ntDispTab("Ball Grabber")
            .add("Ball Grabber Duty Cycle", this::getDutyCycle)    
            .add("Ball grabber motor current", grabberMotor::getOutputCurrent);
    }

    public void runIntake(){
        if (grabberMotor.getOutputCurrent() < MAX_CURRENT_RUNNING){
            grabberMotor.set(INTAKE_SPEED);
        } else {
            grabberMotor.set(0.0);
        }
    }

    public void runOuttake(){
        if (grabberMotor.getOutputCurrent() < MAX_CURRENT_RUNNING){
            grabberMotor.set(OUTTAKE_SPEED);
        } else {
            grabberMotor.set(0.0);
        }
    }

    public void stop() {
        grabberMotor.set(0);
    }

    public double getDutyCycle() {
        return grabberMotor.get();
    }
}
