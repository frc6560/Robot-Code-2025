package com.team6560.frc2025.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;


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
        
        while (true) {
            double currentelevepos = m_leftElev.getPosition().getValueAsDouble();
            //elevator motor position in rotations as double

            if (currentelevepos < targetrotelev - 0.1) {
                m_leftElev.set(0.1);
                m_rightElev.set(0.1);
            }

            if (currentelevepos > 0.1 + targetrotelev) {
                m_leftElev.set(-0.1);
                m_rightElev.set(-0.1);
            }

            else {
                m_rightElev.stopMotor();
                m_rightElev.stopMotor();
            }
        }
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

    //PLACEHOLDER

    public double getElevatorHeight(){
        return 0.0;
    }

    public void resetEncoderPos(double setposition) {
        m_leftElev.setPosition(setposition);
        m_rightElev.setPosition(setposition);
    }
}