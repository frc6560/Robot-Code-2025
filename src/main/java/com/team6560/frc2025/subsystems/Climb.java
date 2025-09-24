package com.team6560.frc2025.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6560.frc2025.Constants.ClimbConstants;
import com.team6560.frc2025.controls.XboxControls;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

// range of motion: 125.96 deg
// GR: 1.5 --> motor spins 188.94 deg

public class Climb extends SubsystemBase {
    
    // public enum State {
    //     UP,
    //     DOWN,
    //     STATIC
    // };

    // motors are linked up in reverse
    private TalonFX motor1;
    private TalonFX motor2;

    // encoder moves in same direction as motor1
    private CANcoder absoluteEncoder;
    private TalonFXConfiguration fxConfig;

    // negative values handled in function logic
    private static final double CLIMB_UP_PERCENT = 0.8;
    private static final double CLIMB_DOWN_PERCENT = 0.8;

    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Climb");
    private final NetworkTableEntry ntPos = ntTable.getEntry("Position");
    private final NetworkTableEntry ntUseSoftlimits = ntTable.getEntry("ntUseSoftlimits");

    // public State state;

    public Climb(XboxControls controls) { 

        this.motor1 = new TalonFX(ClimbConstants.MOTOR_1_ID, "Canivore");
        this.motor2 = new TalonFX(ClimbConstants.MOTOR_2_ID, "Canivore");
        this.absoluteEncoder = new CANcoder(ClimbConstants.CANCODER_ID, "Canivore");

        this.fxConfig = new TalonFXConfiguration(); 
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
 
        motor1.getConfigurator().apply(fxConfig);
        motor2.getConfigurator().apply(fxConfig);
        ntPos.setDouble(this.getEncoderPos());
        ntUseSoftlimits.setBoolean(true);

        // this.state = State.STATIC;

        // should be reset in up position
        // resetEncoderPos();
    }

    @Override
    public void periodic() {
        ntPos.setDouble(this.getEncoderPos());
    }

    public void resetEncoderPos() {
        this.motor1.setPosition(0);
        this.motor2.setPosition(0);
        this.absoluteEncoder.setPosition(0);
    }

    public double getMotor1Pos() {
        return this.motor1.getPosition().getValueAsDouble();
    }

    public double getMotor2Pos() { // reversed
        return -1 * this.motor2.getPosition().getValueAsDouble();
    }

    public double getEncoderPos() {
        return this.absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getSpeed() {
        return this.motor1.get();
    }

    public void stop() {
        this.motor1.set(0);
        this.motor2.set(0);
    }

    // this is just tank drive
    public void down() {
        if (!this.ntUseSoftlimits.getBoolean(true) || this.getEncoderPos() > ClimbConstants.LOWER_SOFT_BOUND) {
            this.motor1.set(-CLIMB_DOWN_PERCENT);
            this.motor2.set(CLIMB_DOWN_PERCENT); } else {
                stop();
            }
        // } else {
        //     this.state = State.STATIC;
        // }
    }

    public void up() {
        if (!this.ntUseSoftlimits.getBoolean(true) || this.getEncoderPos() < ClimbConstants.UPPER_SOFT_BOUND) {
            this.motor1.set(CLIMB_UP_PERCENT);
            this.motor2.set(-CLIMB_UP_PERCENT); } else {
                stop();
                        }
        // } else {
        //     this.state = State.STATIC;
        // }
    }
}