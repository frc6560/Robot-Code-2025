// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6560.frc2025.Constants.WristConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;

import javax.swing.text.Position;


public class Wrist extends SubsystemBase {

  // motor
  private final TalonFX WristMotor;

  // sensors
  private final DigitalInput limitSwitch;

  // encoder stuff
  private CANcoder relativeEncoder;
  private double initialEncoderPos;

  /** Creates a new Wrist. */
  public Wrist() {
      this.WristMotor = new TalonFX(WristConstants.KRAKEN_ID);
      initialEncoderPos = relativeEncoder.getPosition().getValueAsDouble();

      this.limitSwitch = new DigitalInput(WristConstants.LIMSWITCH_ID); //random
      this.relativeEncoder = new CANcoder(WristConstants.CC_ID); //random

      // PID
      var wristPIDController = new Slot0Configs();
      //all random
      wristPIDController.kS = 0.1;
      wristPIDController.kP = 0;
      wristPIDController.kI = 0;
      wristPIDController.kD = 0;

      // Telemetry
      ntDispTab("Wrist")
            .add("Wrist angle", this::getWristAngle)
            .add("Limit switch", this::LimitDown)  
            .add("Soft upper limit", this::getUpperBound)
            .add("Soft bottom limit", this::getLowerBound)
            .add("Overshot limits", this::getOvershoot);
  }

  @Override
  public void periodic() {
    //don't hard code into periodic yet !!
  }

  /** Checks if the wrist is down based on the limit switch. */
  private boolean LimitDown(){
    return !(limitSwitch.get());
  }

  /** Gets the current wrist angle */
  public double getWristAngle(){
    return ((relativeEncoder.getPosition().getValueAsDouble() * 360) - (this.initialEncoderPos * 360) / WristConstants.GEAR_RATIO);
  }

  /** Gets the upper bound of the wrist. Static value defined in Constants. */
  public double getUpperBound(){
    return WristConstants.UPPER_SOFT_BOUND;
  }

  public double getLowerBound(){
    return WristConstants.LOWER_SOFT_BOUND;
  }

  public boolean getOvershoot(){
    double position = this.getWristAngle();
    return(position > WristConstants.UPPER_SOFT_BOUND || position < WristConstants.LOWER_SOFT_BOUND);
  }

  /** Actually sets wrist position. */
  public void SetMotorPosition(double position){
    double currentPosition = this.getWristAngle();

    double deltaPos = (position-currentPosition) / 360 * WristConstants.GEAR_RATIO; //hopefully this conversion factor is correct.
    
    final PositionVoltage m_request = new PositionVoltage(deltaPos);
    WristMotor.setControl(m_request);
  }

  public void overshootHandler(){
    if(getOvershoot()){
      WristMotor.setControl(new PositionVoltage(0));
    }
  }
}