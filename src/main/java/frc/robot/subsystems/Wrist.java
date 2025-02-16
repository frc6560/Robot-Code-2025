// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.WristConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degree;
import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;



public class Wrist extends SubsystemBase {

  // motor
  private final TalonFX m_WristMotor;

  // sensors
  private final DigitalInput m_limitSwitch;

  // encoder stuff
  private CANcoder m_relativeEncoder;
  private double initialEncoderPos;
  private TalonFXConfiguration fxConfig;

  public enum State{
    PICKUP,
    STOW,
    L1,
    L2,
    L4,
    GrabberL2,
    GrabberL4,
    MOVING // placeholder bad state
  };

  private State state;

  private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Wrist");
  private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
  private final NetworkTableEntry ntPosition = ntTable.getEntry("wrist position");
  private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");

  private double targetPos = 0;

  /** Creates a new Wrist. */
  public Wrist() {
      // Initializes motors and encoders
      this.m_WristMotor = new TalonFX(WristConstants.M_ID, "Canivore");
      this.m_relativeEncoder = new CANcoder(WristConstants.CANCODER_ID, "Canivore");

      this.m_WristMotor.setPosition(getWristAngle() / 3.33333);
      initialEncoderPos = 0;
      // m_relativeEncoder.setPosition(initialEncoderPos * );


      this.m_limitSwitch = new DigitalInput(WristConstants.SWITCH_ID); //random
      this.state = State.STOW;


      // Applies the cancoder to the wrist motor
      this.fxConfig = new TalonFXConfiguration();
      // fxConfig.Feedback.FeedbackRemoteSensorID = m_relativeEncoder.getDeviceID();
      // fxConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      fxConfig.Feedback.RotorToSensorRatio = 108;
      fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      m_WristMotor.getConfigurator().apply(fxConfig);

      // PID
      Slot0Configs wristPIDController = new Slot0Configs();
      // all random
      wristPIDController.kS = 0;
      wristPIDController.kG = 0;

      wristPIDController.kP = 1.2;
      wristPIDController.kI = 0.014;
      wristPIDController.kD = 0;


      m_WristMotor.getConfigurator().apply(wristPIDController);

      // Telemetry using shuffleboard's display tab. See NtValueDisplay.
      // ntDispTab("Wrist")
      //       .add("Wrist angle", this::getWristAngle)
      //       .add("Wrist angular velocity", this::getWristVelocity)
      //       .add("Limit switch", this::limitDown)  
      //       .add("Soft upper limit", this::getUpperBound)
      //       .add("Soft bottom limit", this::getLowerBound)
      //       .add("Overshot bounds", this::getOvershoot)
      //       .add("State", ()-> {return this.state.toString();});

      
      ntAngle.setDouble(0.0);
      ntTargetPos.setDouble(this.targetPos);
      ntPosition.setDouble(m_WristMotor.getPosition().getValueAsDouble());
  }



  /** WPILib default periodic function. leave empty. */
  @Override 
  public void periodic() {
    updateNTTable();
    // if (limitDown()){
    //   System.out.println("Limit down");
    // }
  }

  public void updateNTTable(){
      ntAngle.setDouble(getWristAngle());
      ntTargetPos.setDouble(this.targetPos);
      ntPosition.setDouble(m_WristMotor.getPosition().getValueAsDouble());
  }
  /** Checks if the wrist is down based on the limit switch. */
  private boolean limitDown(){
    return m_limitSwitch.get();
  }

  /** Gets the current wrist angle */
  public double getWristAngle(){
    return m_relativeEncoder.getPosition().getValueAsDouble() * 360 / 6;
  }

  public double getWristPosition(){
    return m_WristMotor.getPosition().getValueAsDouble();
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

  /** Updates and fetches the state of the wrist.  */
  public State getState(){
    final double tolerance = 2.5; // degrees
    if(getWristVelocity() > 0.1){
      this.state = State.MOVING;

    } else{
      double angle = getWristAngle();

      if (Math.abs(angle - WristConstants.WristStates.STOW) < tolerance){
        this.state = State.STOW;

      } else if(Math.abs(angle - WristConstants.WristStates.PICKUP) < tolerance){
        this.state = State.PICKUP;

      } else if (Math.abs(angle - WristConstants.WristStates.L1) < tolerance){
        this.state = State.L1;

      } else if (Math.abs(angle - WristConstants.WristStates.L2) < tolerance){
        this.state = State.L2;

      } else if (Math.abs(angle - WristConstants.WristStates.L4) < tolerance){
        this.state = State.L4;

      } else {
        this.state = State.MOVING;
      }

    }
    
    return this.state;
  }

  /* Does not actually set wrist position. Sets encoder position instead.  */
  public void setEncoderPosition(double pos){
    this.m_relativeEncoder.setPosition(0.0);
    this.m_WristMotor.setPosition(0.0);
  } 

  /* Actually sets wrist position. */
  public void setMotorPosition(double position){
    position = Math.min(Math.max(position, WristConstants.LOWER_SOFT_BOUND), WristConstants.UPPER_SOFT_BOUND);
    this.targetPos = position;

    double targetPos = (position) / 360 * 108; //hopefully this conversion factor is correct.

    // 1 rot      5 rot        18                       
    // 360        1 rot        1 rot

    final PositionVoltage m_request = new PositionVoltage(targetPos);
    m_WristMotor.setControl(m_request);
  }

  public void stopMotor(){
    m_WristMotor.set(0);
  }


  // all code below this point is for testing purposes only
  public void turnOnMotor(){
    m_WristMotor.set(0.1);
  }

  public void revMotor(){
    m_WristMotor.set(-0.1);
  }
  
  public void turnOnMotorWithPID(){
    m_WristMotor.setControl(new VelocityVoltage(200));
  }

  public void testMotor(double output){
    m_WristMotor.set(output);
  }
}