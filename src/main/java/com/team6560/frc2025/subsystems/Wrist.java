// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.team6560.frc2025.Constants.WristConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;



public class Wrist extends SubsystemBase {

  // motor
  private final TalonFX WristMotor;

  // sensors
  private final DigitalInput limitSwitch;

  // encoder stuff
  private CANcoder relativeEncoder;
  private double initialEncoderPos;
  private TalonFXConfiguration fxConfig;

  /** Creates a new Wrist. */
  public Wrist() {
      this.WristMotor = new TalonFX(WristConstants.M_ID);
      initialEncoderPos = relativeEncoder.getPosition().getValueAsDouble();

      this.limitSwitch = new DigitalInput(WristConstants.SWITCH_ID); //random
      this.relativeEncoder = new CANcoder(WristConstants.CC_ID); //random

      // applies the cancoder to the wrist motor
      fxConfig.Feedback.FeedbackRemoteSensorID = relativeEncoder.getDeviceID();
      fxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      WristMotor.getConfigurator().apply(fxConfig);

      // PID
      var wristPIDController = new Slot0Configs();
      //all random
      wristPIDController.kS = 0.1;
      wristPIDController.kP = 0;
      wristPIDController.kI = 0;
      wristPIDController.kD = 0;

      WristMotor.getConfigurator().apply(wristPIDController);

      // Telemetry
      ntDispTab("Wrist")
            .add("Wrist angle", this::getWristAngle)
            .add("Limit switch", this::LimitDown)  
            .add("Soft upper limit", this::getUpperBound)
            .add("Soft bottom limit", this::getLowerBound)
            .add("Overshot limits", this::getOvershoot);
  }

  /** WPILib default periodic function. leave empty. */
  @Override 
  public void periodic() {
    overshootHandler();
  }

  /** Checks if the wrist is down based on the limit switch. */
  private boolean LimitDown(){
    return limitSwitch.get();
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

  public void stopMotor(){
    WristMotor.setControl(new PositionVoltage(0));
  }

  public void overshootHandler(){
    if(getOvershoot()){
      stopMotor();
    }
  }
}