package com.team6560.frc2025.autonomous;

import com.team6560.frc2025.Constants.FieldConstants;
import com.team6560.frc2025.commands.AutoAlignCommand;
import com.team6560.frc2025.commands.IntakeCommand;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class AutoFactory {
    private final DriverStation.Alliance alliance;

    private Wrist wrist;
    private Elevator elevator;
    private SwerveSubsystem drivetrain;
    private PipeGrabber grabber;

    public AutoFactory(DriverStation.Alliance alliance, Wrist wrist, Elevator elevator, SwerveSubsystem drivetrain, PipeGrabber grabber) {
        this.alliance = alliance;
        this.wrist = wrist;
        this.elevator = elevator;
        this.drivetrain = drivetrain;
        this.grabber = grabber;
    }

    /** These are functions for returning different autonomous routines. See AutoRoutines.java for more information. */

    private static final Command IDLE= Commands.idle();

    /** These literally do nothing. As in, nothing. */
    Pair<Pose2d, Command> getNoAutoLeft(){
        return Pair.of(FieldConstants.getFarLeft(alliance), IDLE);
    }

    Pair<Pose2d, Command> getNoAutoRight(){
        return Pair.of(FieldConstants.getFarRight(alliance), IDLE);
    }

    Pair<Pose2d, Command> getFourPieceRight(){
        return Pair.of(
            FieldConstants.getRight(alliance),
            Commands.sequence(
                new AutoAlignCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, grabber, PickupLocations.RIGHT_RED),
                new AutoAlignCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, grabber, PickupLocations.RIGHT_RED),
                new AutoAlignCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L2, true),
                new IntakeCommand(wrist, elevator, drivetrain, grabber, PickupLocations.LEFT_RED),
                new AutoAlignCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L2, true)
            )
        );
    }

    Pair<Pose2d, Command> getFourPieceLeft(){
        return null;
    }
}
