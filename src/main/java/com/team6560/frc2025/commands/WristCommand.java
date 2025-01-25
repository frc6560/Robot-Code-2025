// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.controls.ManualControls;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;

/** A command that controls the Wrist subsystem */
public class WristCommand extends Command{
    final Wrist wrist;
    final ManualControls controls;

    final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Wrist");

    public static interface Controls{
        boolean setStow();
        boolean setIntake();

        boolean setLow();
        boolean setHigh();
    }

    public WristCommand(Wrist wrist, ManualControls controls){
        this.wrist = wrist;
        this.controls = controls;

        addRequirements(wrist);
    }

    // Called when the command is initially scheduled. (leave empty)
    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(controls.getSetStow()){
            wrist.SetMotorPosition(WristConstants.STOW_ANGLE);
        } else if (controls.getSetIntake()){
            wrist.SetMotorPosition(WristConstants.INTAKE_ANGLE);
        } else if (controls.getSetLowLevel()){
            wrist.SetMotorPosition(WristConstants.L2_ANGLE);
        } else if (controls.getSetHighLevel()){
            wrist.SetMotorPosition(WristConstants.L4_ANGLE);
        } else{
            wrist.stopMotor();
        }
    }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
