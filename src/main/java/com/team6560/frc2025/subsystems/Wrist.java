// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team6560.frc2025.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.team6560.frc2025.Constants;
import com.team6560.frc2025.Constants.WristConstants;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;


public class Wrist extends SubsystemBase {

  // motor
  private final TalonFX WristMotor;

  // sensors
  private final DigitalInput limitSwitch;

  // relative encoder 
  private final Encoder encoder;

  private double initialEncoderPos;

  /** Creates a new Wrist. */
  public Wrist() {
      this.WristMotor = new TalonFX(WristConstants.KRAKEN_ID);
      initialEncoderPos = WristMotor.getPosition().getValueAsDouble();

      //fix channels
      this.limitSwitch = new DigitalInput(0);
      this.encoder = new Encoder(0, 1);

      // configs gear ratio
      var wristGR = new FeedbackConfigs();
      wristGR.SensorToMechanismRatio = WristConstants.GEAR_RATIO;

      //PID
      var wristPIDController = new Slot0Configs();
      wristPIDController.kS = 0;
      wristPIDController.kP = 0;
      wristPIDController.kI = 0;
      wristPIDController.kD = 0;

      // Network Tables
      ntDispTab("Hood ")
            .add("Wrist Angle", this::getWristAngle)
            .add("Limit switch", this::LimitDown)  
            .add("Soft limit", this::getUpperBound);
  }

  @Override
  public void periodic() {
    // redundancy to check for limit switch stuff
    if (limitSwitch.get()) {
      WristMotor.setControl(new PositionVoltage(0)); // Stop motor immediately
    }
  }

  private boolean LimitDown(){
    return !(limitSwitch.get());
  }

  public double getWristAngle(){
    return ((WristMotor.getPosition().getValueAsDouble() * 360) - (this.initialEncoderPos * 360) / WristConstants.GEAR_RATIO);
  }

  public double getUpperBound(){
    return WristConstants.SOFT_BOUND;
  }

  // actually sets wrist position
  public void SetMotorPosition(double position){
    position = position / 360 * WristConstants.GEAR_RATIO; //hopefully this conversion factor is correct.
    
    final PositionVoltage m_request;
    if(limitSwitch.get()){
      m_request = new PositionVoltage(0);
    }
    else{
      m_request = new PositionVoltage(position);
    }
    WristMotor.setControl(m_request);
  }
}