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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;


public class Wrist extends SubsystemBase {

  // motor
  private final TalonFX WristMotor;

  // sensors
  private final DigitalInput limitSwitch;

  // initial encoder stuff
  private double initialEncoderPos;

  /** Creates a new Wrist. */
  public Wrist() {
      this.WristMotor = new TalonFX(WristConstants.KRAKEN_ID);
      initialEncoderPos = WristMotor.getPosition().getValueAsDouble();

      this.limitSwitch = new DigitalInput(0); //random

      // PID
      var wristPIDController = new Slot0Configs();
      //all random
      wristPIDController.kS = 0;
      wristPIDController.kP = 0;
      wristPIDController.kI = 0;
      wristPIDController.kD = 0;

      // Telemetry
      ntDispTab("Hood ")
            .add("Wrist angle", this::getWristAngle)
            .add("Limit switch", this::LimitDown)  
            .add("Soft limit", this::getUpperBound);
  }

  @Override
  public void periodic() {
    // Redundancy to check for limit switch stuff
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

  /** Actually sets wrist position. */
  public void SetMotorPosition(double position){
    double currentPosition = this.getWristAngle();

    double deltaPos = (position-currentPosition) / 360 * WristConstants.GEAR_RATIO; //hopefully this conversion factor is correct.
    
    final PositionVoltage m_request;
    if(limitSwitch.get()){
      m_request = new PositionVoltage(0);
    }
    else{
      m_request = new PositionVoltage(deltaPos);
    }
    WristMotor.setControl(m_request);
  }
}