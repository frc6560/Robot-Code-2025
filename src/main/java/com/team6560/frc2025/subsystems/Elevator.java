package com.team6560.frc2025.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;


public class Elevator extends SubsystemBase {
    
    private final TalonFX m_leftElev;
    private final TalonFX m_rightElev;

    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    public Elevator() {
        this.m_leftElev = new TalonFX(ElevatorConstants.ELEV_LEFT_ID);
        this.m_rightElev = new TalonFX(ElevatorConstants.ELEV_RIGHT_ID);

        this.topLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_UPPER_LIMIT_SWITCH_ID);
        this.bottomLimitSwitch = new DigitalInput(ElevatorConstants.ELEV_LOWER_LIMIT_SWITCH_ID);

        Slot0Configs elevatorPID = new Slot0Configs();
        // all random
            elevatorPID.kS = 2;
            elevatorPID.kG = 0;

            elevatorPID.kP = 0.002;
            elevatorPID.kI = 0.01;
            elevatorPID.kD = 0.2;
        
        m_leftElev.getConfigurator().apply(elevatorPID);
        m_rightElev.getConfigurator().apply(elevatorPID);


        ntDispTab("Elevator")
            .add("Elevator Height", this::getElevatorHeight)
            .add("Elevator angular velocity", this::getElevatorVelocity)
            .add("Upper limit switch", this::topLimitSwitchDown)
            .add("Bottom limit switch", this::bottomLimitSwitchDown);
    }

    @Override
    public void periodic() {
        checkElevatorBounds();
    }

    public void setelevpos(int posnum) {
        double targetrotelev = 0;

        if (posnum == 1) {
            targetrotelev = 0;
        }
        else if (posnum == 2) {
            targetrotelev = 3;
        }
        else if (posnum == 3) {
            targetrotelev = 6;
        }
        else if (posnum == 4) {
            targetrotelev = 9;
        }

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
        return m_leftElev.getPosition().getValueAsDouble() * ElevatorConstants.ELEV_GEAR_RATIO;
    }

    public double getElevatorVelocity(){
        return m_leftElev.getVelocity().getValueAsDouble();
    }

    public void resetEncoderPos(double setposition) {
        m_leftElev.setPosition(setposition);
        m_rightElev.setPosition(setposition);
    }

    public void checkElevatorBounds() {
        if (topLimitSwitchDown() || bottomLimitSwitchDown()) {
            stopMotors();;
        }
    }

    // all code below this point is for testing purposes
    public void turnOnMotors(){
        m_leftElev.setControl(new VelocityVoltage(0.2));
        m_rightElev.setControl(new VelocityVoltage(0.2));
    }

    public void turnOnMotorsNoPID(){
        m_leftElev.set(0.1);
        m_rightElev.set(0.1);
    }
}