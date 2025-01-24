package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.controls.ManualControls;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.Command;

/** A command that controls the Wrist subsystem */
public class WristCommand {
    final Wrist wrist;
    final ManualControls controls;

    final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Wrist");

    public static interface Controls{
        boolean setStowPos();
        boolean setIntakePos();

        boolean setL2Pos();
        boolean setL4Pos();
    }

    public WristCommand(Wrist wrist, ManualControls controls){
        this.wrist = wrist;
        this.controls = controls;
    }
}