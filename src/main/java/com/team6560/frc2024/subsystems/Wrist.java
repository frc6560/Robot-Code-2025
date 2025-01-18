// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team6560.frc2024.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.team6560.frc2024.Constants.WristConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Wrist extends SubsystemBase {
  // motor
  private final TalonFX WristMotor;


  // sensors
  private final DigitalInput limitSwitch;

  // relative encoder 
  private final DigitalInput input;
  private final DutyCycleEncoder encoder;

  /** Creates a new Wrist. */
  public Wrist() {
      this.WristMotor = new TalonFX(WristConstants.KRAKEN_ID);
      this.limitSwitch = new DigitalInput(0);

      this.input = new DigitalInput(0); //fix ports according to constants file
      this.encoder = new DutyCycleEncoder(input);

      // configs gear ratio
      var wristGR = new FeedbackConfigs();
      wristGR.SensorToMechanismRatio = WristConstants.GEAR_RATIO;

      //PID
      var wristPIDController = new Slot0Configs();
      //do we need kS?
      wristPIDController.kS = 0;

      wristPIDController.kP = 0;
      wristPIDController.kI = 0;
      wristPIDController.kD = 0;


      
  }

  @Override
  public void periodic() {
     
  }

  // setting wrist velocity logic
  public void SetMotorSpeed(double speed){
    if(limitSwitch.get()){
      WristMotor.set(0);
    }
    else{
      WristMotor.set(speed);
    }
  }
}
