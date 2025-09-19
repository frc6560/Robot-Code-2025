package com.team6560.frc2025.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallGrabber extends SubsystemBase {
    
    private SparkMax grabberMotor;
    private static final int GRABBER_MOTOR_ID = 1;
    
    private static final double INTAKE_SPEED = -0.3;
    private static final double OUTTAKE_SPEED = 0.7;
    
    // Current-based detection
    private static final double MAX_CURRENT_RUNNING = 30;
    
    // Voltage-based detection constants
    private static final double MIN_VOLTAGE_THRESHOLD = 8.0; // Voltage drops when motor stalls
    private static final double VOLTAGE_DROP_THRESHOLD = 2.0; // How much voltage can drop from battery voltage
    
    public BallGrabber() {
        this.grabberMotor = new SparkMax(GRABBER_MOTOR_ID, MotorType.kBrushless);
        ntDispTab("Ball Grabber")
            .add("Ball Grabber Duty Cycle", this::getDutyCycle)
            .add("Ball grabber motor current", grabberMotor::getOutputCurrent)
            .add("Ball grabber bus voltage", this::getBusVoltage)
            .add("Ball grabber applied voltage", this::getAppliedVoltage)
            .add("Is ball stuck (current)", this::isBallStuckByCurrent)
            .add("Is ball stuck (voltage)", this::isBallStuckByVoltage);
    }
    
    public void runIntake(){
        if (!isBallStuck()) {
            grabberMotor.set(INTAKE_SPEED);
        } else {
            grabberMotor.set(0.0);
        }
    }
    
    public void runOuttake(){
        if (!isBallStuck()) {
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
    
    // Voltage monitoring methods
    public double getBusVoltage() {
        return grabberMotor.getBusVoltage(); // Battery voltage to the motor controller
    }
    
    public double getAppliedVoltage() {
        return grabberMotor.getAppliedOutput() * getBusVoltage(); // Actual voltage applied to motor
    }
    
    // Detection methods
    public boolean isBallStuckByCurrent() {
        return grabberMotor.getOutputCurrent() >= MAX_CURRENT_RUNNING;
    }
    
    public boolean isBallStuckByVoltage() {
        double busVoltage = getBusVoltage();
        double appliedVoltage = Math.abs(getAppliedVoltage());
        
        // Check if applied voltage is low (indicating stall) or if voltage has dropped significantly
        return appliedVoltage < MIN_VOLTAGE_THRESHOLD || 
               (busVoltage - appliedVoltage) > VOLTAGE_DROP_THRESHOLD;
    }
    
    public boolean isBallStuck() {
        // Use both current and voltage detection for more reliable detection
        return isBallStuckByCurrent() || isBallStuckByVoltage();
    }
}