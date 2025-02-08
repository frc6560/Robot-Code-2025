// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package com.team6560.frc2025.controls;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.team6560.frc2025.utility.PovNumberStepper;

public class ManualControls{
    private XboxController drivingController;
    private XboxController scoringController;

    private NetworkTable wristTable;

    private final PovNumberStepper speed;
    private final PovNumberStepper turnSpeed;  
    
    /** 1 controller implementation */
    public ManualControls(XboxController xbox){
        this(xbox, xbox);
    }

    public ManualControls(XboxController xbox, XboxController controllerStation){
        this.drivingController = xbox;
        this.scoringController = controllerStation;

        this.speed = null;
        this.turnSpeed = null;
        
        wristTable = NetworkTableInstance.getDefault().getTable("Wrist");
    }
    // ---- DRIVETRAIN ---- (copied from 2024 code)

//   @Override
//   public double driveX() {
//     double speedModifier = nitroMode() ? 1.25 : 1;

//     return -modifyAxis(drivingController.getLeftY() * speed.get() * speedModifier);
//   }

//   @Override
//   public double driveY() {
//     double speedModifier = nitroMode() ? 1.25 : 1;

//     return -modifyAxis(drivingController.getLeftX() * speed.get() * speedModifier);
//   }

//   @Override
//   public double driveRotationX() {
//     return -modifyAxis(drivingController.getRightX() * turnSpeed.get());
//   }

//   @Override
//   public boolean driveResetYaw() {
//     return drivingController.getStartButton();
//   }

//   @Override
//   public boolean driveResetGlobalPose() {
//     return drivingController.getBackButton();
//   }

//   public boolean nitroMode(){
//     return drivingController.getRightTriggerAxis() > 0.2;
//   }
}