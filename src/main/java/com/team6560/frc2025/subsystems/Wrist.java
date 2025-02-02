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
  private final TalonFX m_WristMotor;

  // sensors
  private final DigitalInput m_limitSwitch;

  // encoder stuff
  private CANcoder m_relativeEncoder;
  private double initialEncoderPos;
  private TalonFXConfiguration fxConfig;

  enum State{
    PICKUP,
    STOW,
    L2,
    L4,
    MOVING // placeholder bad state
  };

  private State state;

  /** Creates a new Wrist. */
  public Wrist() {
      // Initializes motors
      this.m_WristMotor = new TalonFX(WristConstants.M_ID);
      this.m_relativeEncoder = new CANcoder(WristConstants.CANCODER_ID); //random
      initialEncoderPos = m_relativeEncoder.getPosition().getValueAsDouble();

      this.m_limitSwitch = new DigitalInput(WristConstants.SWITCH_ID); //random
      this.state = State.STOW;


      // Applies the cancoder to the wrist motor
      this.fxConfig = new TalonFXConfiguration();
      fxConfig.Feedback.FeedbackRemoteSensorID = m_relativeEncoder.getDeviceID();
      fxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      m_WristMotor.getConfigurator().apply(fxConfig);

      // PID
      var wristPIDController = new Slot0Configs();
      // all random
      wristPIDController.kS = 2;
      wristPIDController.kG = 0;

      wristPIDController.kP = 0.002;
      wristPIDController.kI = 0.01;
      wristPIDController.kD = 0.2;

      m_WristMotor.getConfigurator().apply(wristPIDController);

      // Telemetry using shuffleboard's display tab. See NtValueDisplay.
      ntDispTab("Wrist")
            .add("Wrist angle", this::getWristAngle)
            .add("Wrist angular velocity", this::getWristVelocity)
            .add("Limit switch", this::LimitDown)  
            .add("Soft upper limit", this::getUpperBound)
            .add("Soft bottom limit", this::getLowerBound)
            .add("Overshot", this::getOvershoot)
            .add("State", this::getState);
  }



  /** WPILib default periodic function. leave empty. */
  @Override 
  public void periodic() {
    overshootHandler();
  }

  /** Checks if the wrist is down based on the limit switch. */
  private boolean LimitDown(){
    return m_limitSwitch.get();
  }

  /** Gets the current wrist angle */
  public double getWristAngle(){
    return ((m_relativeEncoder.getPosition().getValueAsDouble() * 360) - (this.initialEncoderPos * 360)) / WristConstants.GEAR_RATIO;
  }

  /** Gets the current wrist velocity. */
  public double getWristVelocity(){
    return (m_relativeEncoder.getVelocity().getValueAsDouble() * 360)/WristConstants.GEAR_RATIO;
  }

  /** Gets the upper bound of the wrist. Static value defined in Constants. */
  public double getUpperBound(){
    return WristConstants.UPPER_SOFT_BOUND;
  }
  /** Gets the lower bound of the wrist. */
  public double getLowerBound(){
    return WristConstants.LOWER_SOFT_BOUND;
  }

  /** Determines if the wrist has overshot its initial angle */
  public boolean getOvershoot(){
    double position = this.getWristAngle();
    return(position > WristConstants.UPPER_SOFT_BOUND || position < WristConstants.LOWER_SOFT_BOUND);
  }

  /** Updates the state of the wrist.  */
  public void handleState(){
    final double tolerance = 5;
    if(getWristVelocity() > 0.1){
      this.state = State.MOVING;
    }
    else{
      double angle = getWristAngle();
      if(angle > WristConstants.STOW_ANGLE - tolerance && angle < WristConstants.STOW_ANGLE + tolerance){
        this.state = State.STOW;
      } else if(angle > WristConstants.INTAKE_ANGLE - tolerance && angle < WristConstants.INTAKE_ANGLE + tolerance){
        this.state = State.PICKUP;
      } else if (angle > WristConstants.L2_ANGLE - tolerance && angle < WristConstants.L2_ANGLE + tolerance){
        this.state = State.L2;
      } else if (angle > WristConstants.L4_ANGLE - tolerance && angle < WristConstants.L4_ANGLE + tolerance){
        this.state = State.L4;
      }
      else this.state = State.MOVING;
    }
  }

  /** Fetches the state of the wrist */
  public State getState(){
    return this.state;
  }

  /** Actually sets wrist position. */
  public void SetMotorPosition(double position){

    double targetPos = (position) / 360 * WristConstants.GEAR_RATIO; //hopefully this conversion factor is correct.
    
    final PositionVoltage m_request = new PositionVoltage(targetPos);
    m_WristMotor.setControl(m_request);
  }



  public void stopMotor(){
    m_WristMotor.setControl(new PositionVoltage(0));
  }

  public void overshootHandler(){
    if(getOvershoot()){
      stopMotor();
    }
  }
}