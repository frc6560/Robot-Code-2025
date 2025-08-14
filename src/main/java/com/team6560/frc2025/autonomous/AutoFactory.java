package com.team6560.frc2025.autonomous;

import com.team6560.frc2025.Constants.FieldConstants;
import com.team6560.frc2025.commands.automations.IntakeCommand;
import com.team6560.frc2025.commands.automations.ScoreCommand;
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
    private DriverStation.Alliance alliance;

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

    /** These are four piece autos for various situations. All start on the left/right sides. */
    Pair<Pose2d, Command> getFourPieceBackRight(){
        return Pair.of(
            FieldConstants.getRight(alliance),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4, true)
            )
        );
    }

    Pair<Pose2d, Command> getFourPieceBackLeft(){
        return Pair.of(
            FieldConstants.getLeft(alliance),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.BOTTOM_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4, true)
            )
        );
    }

    Pair<Pose2d, Command> getFourPieceRight(){
        return Pair.of(
            FieldConstants.getRight(alliance),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true)
            )
        );
    }

    Pair<Pose2d, Command> getFourPieceLeft(){
        return Pair.of(
            FieldConstants.getLeft(alliance),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.BOTTOM_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.BOTTOM_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4, true)
            )
        );
    }

    Pair<Pose2d, Command> getThreePieceBackRight(){
        return Pair.of(
            FieldConstants.getFarRight(alliance),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4, true)
            )
        );
    }

    /** Scores one piece to the side, and two to the back */
    Pair<Pose2d, Command> getThreePieceBackLeft(){
        return Pair.of(
            FieldConstants.getFarLeft(alliance),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4, true)
            )
        );
    }

    /** Scores 2.5 pieces in the center. */
    Pair<Pose2d, Command> getThreePieceCenter(){
        return Pair.of(
            FieldConstants.getCenter(alliance),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.FAR_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT)
            )
        );
    }

    Pair<Pose2d, Command> getTest(){
        return Pair.of(
            new Pose2d(11.644, 6.096, FieldConstants.getCenter(alliance).getRotation()),
            Commands.sequence(
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.TEST),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.LEFT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
                new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.TEST),
                new ScoreCommand(wrist, elevator, grabber, drivetrain, ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L3, true)
            )
        );
    }

    public void updateAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }
}
