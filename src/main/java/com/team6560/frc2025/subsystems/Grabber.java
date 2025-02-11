package com.team6560.frc2025.subsystems;
import com.revrobotics.spark.SparkMax;

import com.team6560.frc2025.Constants.GrabberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.core.CoreCANrange;


public class Grabber extends SubsystemBase {

    private final SparkMax GMOTOR;
    private final CoreCANrange CANrangedistance;
    private final double DISTANCE_THRESHOLD = 1.0;     
    public Grabber() {
    this.GMOTOR = new SparkMax(GrabberConstants.GRABBER_MOTOR_ID, SparkMax.MotorType.kBrushless);

    this.CANrangedistance = new CoreCANrange(1); // Use the actual CAN ID of your sensor

    }

    public void grab() {
        double distance = CANrangedistance.getDistance().getValueAsDouble(); // fixed

        if (distance > DISTANCE_THRESHOLD) {
            GMOTOR.set(0.1);
        } else {    
            stop();
        }
    }

    public void release() {
        GMOTOR.set(-0.1);
    }
    public void stop() {
        GMOTOR.set(0);
    }
   
    @Override
    public void periodic() {
        grab(); // Automatically runs the grab in periodic liewke signals     
    } 

    // all code below this point is for testing purposes only
    public void turnOn() {
        GMOTOR.set(0.1);
    }
}