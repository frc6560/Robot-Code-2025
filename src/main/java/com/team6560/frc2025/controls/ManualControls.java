package com.team6560.frc2025.controls;

import com.team6560.frc2025.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.MathUtil;

import static com.team6560.frc2025.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class ManualControls{
    private XboxController drivingController;
    private XboxController scoringController;

    private NetworkTable wristTable;
    
    /** 1 controller implementation */
    public ManualControls(XboxController xbox){
        this(xbox, xbox);
    }

    public ManualControls(XboxController xbox, XboxController controllerStation){
        this.drivingController = xbox;
        this.scoringController = controllerStation;

        ntDispTab("Controls")
            .add("Y Joystick", this::driveY)
            .add("X Joystick", this::driveX)
            .add("Rotation Joystick", this::driveRotationX);
            
    }
    // ---- DRIVETRAIN ---- (copied from 2024 code)

  @Override
  public double driveX() {
    double speedModifier = nitroMode() ? 1.25 : 1;

    return -modifyAxis(drivingController.getLeftY() * speed.get() * speedModifier);
  }

  @Override
  public double driveY() {
    double speedModifier = nitroMode() ? 1.25 : 1;

    return -modifyAxis(drivingController.getLeftX() * speed.get() * speedModifier);
  }

  @Override
  public double driveRotationX() {
    return -modifyAxis(drivingController.getRightX() * turnSpeed.get());
  }

  @Override
  public boolean driveResetYaw() {
    return drivingController.getStartButton();
  }

  @Override
  public boolean driveResetGlobalPose() {
    return drivingController.getBackButton();
  }

  public boolean nitroMode(){
    return drivingController.getRightTriggerAxis() > 0.2;
  }
}