package com.team6560.frc2025.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6560.frc2025.Constants.ElevatorConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
public class Elevator extends SubsystemBase {

    public enum State {
        STOW,
        L2,
        L3,
        L4,
        BALL,
        S_L2,
        S_L3, 
        S_L4, S_STOW
    };
    
    private final TalonFX m_leftElev;
    private final TalonFX m_rightElev;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    private double targetPos = 0;

    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Elevator");
    private final NetworkTableEntry ntHeight = ntTable.getEntry("Height");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target height");

    public Elevator() {
        this.m_leftElev = new TalonFX(ElevatorConstants.ELEV_LEFT_ID, "Canivore");
        this.m_rightElev = new TalonFX(ElevatorConstants.ELEV_RIGHT_ID, "Canivore");

        this.topLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_UPPER_LIMIT_SWITCH_ID);
        this.bottomLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_LOWER_LIMIT_SWITCH_ID);

        Slot0Configs elevatorPID = new Slot0Configs();

        elevatorPID.kS = 0;
        elevatorPID.kG = 0.4; // 0.4

        elevatorPID.kP = 0.7; // 0.7
        elevatorPID.kI = 0.01; // this is the integral term, which is used to eliminate steady-state error
        elevatorPID.kD = 0; 

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        m_leftElev.getConfigurator().apply(config.withSlot0(elevatorPID));
        m_rightElev.getConfigurator().apply(config.withSlot0(elevatorPID));

        ntHeight.setDouble(0.0);
        ntTargetPos.setDouble(0.0);
        
    }

    @Override
    public void periodic() {
        updateNTTable();
    }

    public void updateNTTable(){
        ntHeight.setDouble(getElevatorHeight());
        ntTargetPos.setDouble(this.targetPos);

    }

    public void setElevatorPosition(double targetrotelev) {
        this.targetPos = targetrotelev;
        final PositionVoltage m_request = new PositionVoltage(targetrotelev);

        m_leftElev.setControl(m_request);
        m_rightElev.setControl(m_request);
    }


    public void stopMotors() {
        m_rightElev.stopMotor();
        m_leftElev.stopMotor();
    }

    public boolean topLimitSwitchDown() {
        return topLimitSwitch.get();
    }

    public boolean bottomLimitSwitchDown() {
        return bottomLimitSwitch.get();
    }

    public double getElevatorHeight(){
        return m_leftElev.getPosition().getValueAsDouble();// * ElevatorConstants.ELEV_GEAR_RATIO;
    }

    public double getElevatorVelocity(){
        return m_leftElev.getVelocity().getValueAsDouble();
    }

    public void resetEncoderPos(double setposition) {
        m_leftElev.setPosition(setposition);
        m_rightElev.setPosition(setposition);
    }

    // all code below this point is for testing purposes
    // public void turnOnMotors(){
    //     m_leftElev.setControl(new VelocityVoltage(200));
    //     m_rightElev.setControl(new VelocityVoltage(200));
    // }
// 
    // public void turnOnMotorsNoPID(){
    //     m_leftElev.set(0.1);
    //     m_rightElev.set(0.1);
    // }
    // public void revMotorsNoPID(){
    //     m_leftElev.set(-0.1);
    //     m_rightElev.set(-0.1);
    // }
// 
    // public void testMotor(double output){
    //     m_leftElev.set(output);
    //     m_rightElev.set(output);
    // }
}