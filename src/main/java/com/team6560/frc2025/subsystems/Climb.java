package com.team6560.frc2025.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase{
    private final TalonFX m_LeftClimbMotor;
    private final TalonFX m_RightClimbMotor;

    public Climb(){
        m_LeftClimbMotor = new TalonFX(0);
        m_RightClimbMotor = new TalonFX(0);
        // todo: update in constants 
    }

    public void testMotors(){
        m_LeftClimbMotor.set(0.1);
        m_RightClimbMotor.set(0.1);
    }

    public void stopMotors(){
        m_LeftClimbMotor.set(0);
        m_RightClimbMotor.set(0);
    }
}
