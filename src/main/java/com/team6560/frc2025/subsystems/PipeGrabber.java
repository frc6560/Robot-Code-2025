package com.team6560.frc2025.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PipeGrabber extends SubsystemBase {
    // Initializes all the motors you'll need, along with some constants. 
    private SparkFlex m_grabberMotor; // This is the motor
    private SparkClosedLoopController m_controller;

    private RelativeEncoder m_encoder;


    private static final int GRABBER_MOTOR_ID = 17;
    private static final double INTAKE_SPEED = 0.5;
    private static final double OUTTAKE_SPEED = -0.32;
    private static final double OUTTAKE_SPEED_L1 = -0.29;

    // This is your constructor. Creates a new PipeGrabber.
    public PipeGrabber() {
        this.m_grabberMotor = new SparkFlex(GRABBER_MOTOR_ID, MotorType.kBrushless);
        // This is for telemetry.
        ntDispTab("Grabber")
            .add("Grabber Duty Cycle", this::getDutyCycle);
    
        SparkFlexConfig config = new SparkFlexConfig();

        config.closedLoop
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-0.9, 0.9);

        m_controller = m_grabberMotor.getClosedLoopController();
        m_encoder = m_grabberMotor.getEncoder();
    }

    public void runOuttakePositionBased(){
        m_controller.setReference(-4, ControlType.kPosition);
    }

    // These are the methods that make the grabber run.
    public void runIntake(){
        // This sets the motor's velocity to INTAKE_SPEED
        m_grabberMotor.set(INTAKE_SPEED);
    }

    public void zeroEncoder(){
        m_encoder.setPosition(0);
    }

    public double getEncoder(){
        return m_encoder.getPosition();
    }

    // These follow the same structure as above!
    public void runIntakeMaxSpeed() {
        m_grabberMotor.set(1.0);
    }

    public void runGrabberOuttake(){
        m_grabberMotor.set(OUTTAKE_SPEED);
    }

    public void runGrabberOuttakeL1() {
        m_grabberMotor.set(OUTTAKE_SPEED_L1);
    }

    public void runGrabberOuttakeMaxSpeed() {
        m_grabberMotor.set(-1.0);
    }

    public void stop() {
        m_grabberMotor.set(0);
    }

    public double getDutyCycle() {
        return m_grabberMotor.get();
    }

    public boolean hasGamePiece() {
        // TODO: fix, please.
        return false;
    }
    
    public double getOutputCurrent() {
        return m_grabberMotor.getOutputCurrent();
    }
}
