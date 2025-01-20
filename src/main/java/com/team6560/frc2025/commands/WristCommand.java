package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.Constants.WristConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

/** A command that controls the Wrist subsystem */
public class WristCommand {
    public static interface Controls{
        boolean setStowPos();

        boolean setL2Pos();

        boolean setL4Pos();

        boolean setIntakePos();
    }
    // add controls here

    // private final Wrist Wrist;


}
