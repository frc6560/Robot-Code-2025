package com.team6560.frc2025.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.ctre.phoenix6.controls.PositionVoltage;
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

        ntDispTab("Elevator")
            .add("Elevator Height", this::getElevatorHeight)
            .add("Elevator angular velocity", this::getElevatorVelocity)
            .add("Upper limit switch", this::getUpperBound)
            .add("Bottom limit switch", this::getLowerBound)
            .add("State", this::getState);
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
        m_rightElev.stopMotor();
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
}

// l2: 4.3in
// l3: 20.05in
// 44.425in

//in rotations:
// l2: 1.613
// l3: 7.521
// l4: 16.664