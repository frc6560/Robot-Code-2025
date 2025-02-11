package com.team6560.frc2025.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final TalonFX m_LeftIntakeMotor;
    private final TalonFX m_RightIntakeMotor;

    public Intake(){
        m_LeftIntakeMotor = new TalonFX(0);
        m_RightIntakeMotor = new TalonFX(0);
        // todo: update in constants 
    }

    public void testMotors(){
        m_LeftIntakeMotor.set(0.1);
        m_RightIntakeMotor.set(0.1);
    }

    public void stopMotors(){
        m_LeftIntakeMotor.set(0);
        m_RightIntakeMotor.set(0);
    }
}
