package com.team6560.frc2025.autonomous;

import java.util.Set;

import com.team6560.frc2025.Constants.FieldConstants;
import com.team6560.frc2025.commands.automations.CoralScoreCommandFactory;
import com.team6560.frc2025.commands.automations.IntakeCommand;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


// TODOs: delete all commented out files in robot.java, and this file
public class AutoFactory {
    private DriverStation.Alliance alliance;

    private Wrist wrist;
    private Elevator elevator;
    private SwerveSubsystem drivetrain;
    private PipeGrabber grabber;

    private CoralScoreCommandFactory scoreFactory;

    public AutoFactory(DriverStation.Alliance alliance, Wrist wrist, Elevator elevator, SwerveSubsystem drivetrain, PipeGrabber grabber) {
        if(alliance == null) {
            this.alliance = DriverStation.Alliance.Red;
        }
        else{
            this.alliance = alliance;
        }
        this.wrist = wrist;
        this.elevator = elevator;
        this.drivetrain = drivetrain;
        this.grabber = grabber;

        scoreFactory = new CoralScoreCommandFactory(wrist, elevator, grabber, drivetrain, null);
    }

    /** These are functions for returning different autonomous routines. See AutoRoutines.java for more information. */

    private static final Command IDLE= Commands.idle();

    public Command getResetGyro(Pose2d startPose) {
        return Commands.runOnce(() -> drivetrain.getSwerveDrive().setGyro(new Rotation3d(0, 0, startPose.getRotation().getRadians())), drivetrain);
    }

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
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.TOP_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    } 

    Pair<Pose2d, Command> getFourPieceBackLeft(){
        return Pair.of(
            FieldConstants.getLeft(alliance),
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.BOTTOM_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    Pair<Pose2d, Command> getFourPieceLeft(){
        return Pair.of(
            FieldConstants.getLeft(alliance),
            Commands.defer(() -> Commands.sequence(
                scoreFactory.getScoreAuto(ReefSide.RIGHT, "4pl1", ReefLevel.L4),
                new IntakeCommand(wrist, elevator, drivetrain, "4pl2"),
                scoreFactory.getScoreAuto(ReefSide.LEFT, "4pl3", ReefLevel.L4),
                new IntakeCommand(wrist, elevator, drivetrain, "4pl4"),
                scoreFactory.getScoreAuto(ReefSide.RIGHT, "4pl5", ReefLevel.L4),
                new IntakeCommand(wrist, elevator, drivetrain, "4pl6")
                // scoreFactory.getScoreAuto(ReefSide.LEFT, "4pl7", ReefLevel.L4)
            ), Set.of(wrist, elevator, grabber, drivetrain))
        );
    }

    Pair<Pose2d, Command> getFourPieceRight(){
        return Pair.of(
            FieldConstants.getRight(alliance),
            Commands.defer(() -> Commands.sequence(
                scoreFactory.getScoreAuto(ReefSide.LEFT, "4pr1", ReefLevel.L4),
                new IntakeCommand(wrist, elevator, drivetrain, "4pr2"),
                scoreFactory.getScoreAuto(ReefSide.RIGHT, "4pr3", ReefLevel.L4),
                new IntakeCommand(wrist, elevator, drivetrain, "4pr4"),
                scoreFactory.getScoreAuto(ReefSide.LEFT, "4pr5", ReefLevel.L4),
                new IntakeCommand(wrist, elevator, drivetrain, "4pr6")
            ), Set.of(wrist, elevator, grabber, drivetrain))
        );
    }

    Pair<Pose2d, Command> getThreePieceBackRight(){
        return Pair.of(
            FieldConstants.getFarRight(alliance),
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.RIGHT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    /** Scores one piece to the side, and two to the back */
    Pair<Pose2d, Command> getThreePieceBackLeft(){
        return Pair.of(
            FieldConstants.getFarLeft(alliance),
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.BOTTOM_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.FAR_RIGHT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.FAR_RIGHT, ReefLevel.L4)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    /** Scores 2.5 pieces in the center. */
    Pair<Pose2d, Command> getThreePieceCenter(){
        return Pair.of(
            FieldConstants.getCenter(alliance),
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.FAR_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.LEFT)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    Pair<Pose2d, Command> getTest(){
        return Pair.of(
            FieldConstants.getRight(alliance),
            // Commands.defer(() -> Commands.sequence(
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.TEST),
            //     scoreFactory.getScoreAuto(ReefSide.LEFT, ReefIndex.TOP_LEFT, ReefLevel.L4),
            //     new IntakeCommand(wrist, elevator, drivetrain, PickupLocations.TEST),
            //     scoreFactory.getScoreAuto(ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L3)
            // ), Set.of(wrist, elevator, grabber, drivetrain))
            IDLE
        );
    }

    public void updateAlliance(DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }
}
