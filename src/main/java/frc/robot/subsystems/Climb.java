// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;


public class Climb extends SubsystemBase {
    private final TalonFX climbMotor1;
    private final TalonFX climbMotor2;
    private final double rotations = 1;
    private static final double climbSpeed = 0.3;


    public Climb(){

        this.climbMotor1 = new TalonFX(ClimbConstants.MOTOR1_ID);
        this.climbMotor2 = new TalonFX(ClimbConstants.MOTOR2_ID);

        climbMotor1.getConfigurator().apply(new TalonFXConfiguration()); // keeps motor behavior consistent
        climbMotor2.getConfigurator().apply(new TalonFXConfiguration()); // keeps motor behavior consistent
    }

    public void doClimbUp() {
        climbMotor1.set(climbSpeed);
        climbMotor2.set(climbSpeed);
        if(climbMotor1.getPosition().getValueAsDouble()>=rotations){
            climbMotor1.stopMotor();
            climbMotor2.stopMotor();
        }
    }

    public void doClimbDown() {
        climbMotor1.set(-climbSpeed); // ask alex if negative climb speed works
        climbMotor2.set(-climbSpeed);
        if(climbMotor1.getPosition().getValueAsDouble()<=rotations){
            climbMotor1.stopMotor();
            climbMotor2.stopMotor();
        }
    }


        @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }
}
