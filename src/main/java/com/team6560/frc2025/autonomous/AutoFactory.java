package com.team6560.frc2025.autonomous;

import com.team6560.frc2025.Constants.FieldConstants;
import com.team6560.frc2025.commands.AutoAlignCommand;
import com.team6560.frc2025.commands.IntakeCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoFactory {
    private final DriverStation.Alliance alliance;

    public AutoFactory(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    /** These are functions for returning different autonomous routines. See AutoRoutines.java for more information. */

    private static final Command IDLE= Commands.idle();

    /** These literally do nothing. As in, nothing. */
    Pair<Pose2d, Command> getNoAutoLeft(){
        return Pair.of(FieldConstants.FAR_LEFT_RED, IDLE);
    }

    Pair<Pose2d, Command> getNoAutoRight(){
        return Pair.of(FieldConstants.FAR_RIGHT_RED, IDLE);
    }
}
