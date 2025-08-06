package com.team6560.frc2025;

import com.pathplanner.lib.auto.NamedCommands;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.OperatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.commands.BallGrabberCommand;
import com.team6560.frc2025.commands.ClimbCommand;
import com.team6560.frc2025.commands.ElevatorCommand;
import com.team6560.frc2025.commands.IntakeCommand;
import com.team6560.frc2025.commands.PipeGrabberCommand;
import com.team6560.frc2025.commands.WristCommand;
import com.team6560.frc2025.commands.AutoAlignCommand;
import com.team6560.frc2025.commands.auto.*;
import com.team6560.frc2025.subsystems.BallGrabber;
import com.team6560.frc2025.subsystems.Climb;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;

import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.path.TravelingSalesman;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.File;
import com.frc3481.swervelib.SwerveInputStream;

// took out subsystems + added camera
public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final XboxController firstXbox = new XboxController(0);
  final XboxController secondXbox = new XboxController(1);

  private final ManualControls controls = new ManualControls(firstXbox, secondXbox);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

  private final Climb climb;
  private final ClimbCommand climbCommand;
  private final PipeGrabber pipeGrabber;
  private final PipeGrabberCommand pipeGrabberCommand;
  private final BallGrabber ballGrabber;
  private final BallGrabberCommand ballGrabberCommand;

  private final Wrist wrist;
  private final Elevator elevator = new Elevator();

  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
      () -> (Math.pow(driverXbox.getLeftY(), 2)
            * Math.copySign(1, driverXbox.getLeftY())) 
            * -0.9 * ((firstXbox.getLeftTriggerAxis() > 0.25) || (secondXbox.getLeftBumperButton() || elevator.getElevatorHeight() > ElevatorConstants.ElevatorStates.STOW + 1) ? 0.6 : 1),
      () -> (Math.pow(driverXbox.getLeftX(), 2)
            * Math.copySign(1, driverXbox.getLeftX())) 
            * -0.9 * ((firstXbox.getLeftTriggerAxis() > 0.25) || (secondXbox.getLeftBumperButton() || elevator.getElevatorHeight() > ElevatorConstants.ElevatorStates.STOW + 1) ? 0.6 : 1))
    .withControllerRotationAxis(() -> 
    driverXbox.getRightX() * driverXbox.getRightX() * Math.copySign(1, driverXbox.getRightX()))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);; 

  public RobotContainer() {

    climb = new Climb(controls);
    climbCommand = new ClimbCommand(climb, controls);
    climb.setDefaultCommand(climbCommand);
    
    ballGrabber = new BallGrabber();
    ballGrabberCommand = new BallGrabberCommand(ballGrabber, controls);
    ballGrabber.setDefaultCommand(ballGrabberCommand);

    pipeGrabber = new PipeGrabber();
    pipeGrabberCommand = new PipeGrabberCommand(pipeGrabber, controls);
    pipeGrabber.setDefaultCommand(pipeGrabberCommand);

    wrist = new Wrist();
    wrist.setDefaultCommand(new WristCommand(wrist, controls));
    elevator.setDefaultCommand(new ElevatorCommand(elevator, controls));

    configureBindings();
    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("Test", getProceduralGeneratedAuto());
    autoChooser.addOption("3 Piece Top", get3PieceTopAuto());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  private void configureBindings() { 

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));

    driverXbox.a().onTrue((Commands.runOnce(drivebase::resetOdometryToLimelight)));

    // Use auto align with scoring
    driverXbox.y().whileTrue(Commands.runOnce(() -> new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase,
                              ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, false).schedule(), drivebase));
    driverXbox.b().whileTrue(Commands.runOnce(() -> new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase,
                              ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L3, false).schedule(), drivebase));
    driverXbox.x().whileTrue(Commands.runOnce(() -> new IntakeCommand(wrist, elevator, drivebase, pipeGrabber, 
                              new Pose2d(11.907, 6.120, Rotation2d.fromDegrees(120))).schedule(), drivebase));

  }

  public void elevL4BeginTele() { // values for auto (don't touch!)
    elevator.setElevatorPosition(17.65);
    wrist.setMotorPosition(40.0);
  }

  public void resetLLBeforeAuto() {
    drivebase.resetOdometryToLimelight();
  }

  public Command getProceduralGeneratedAuto() {
    // return Commands.runOnce(() -> new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase,
    // ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, true).schedule(), drivebase)
    // .andThen(Commands.runOnce(() -> new IntakeCommand(wrist, elevator, drivebase, pipeGrabber, 
    // new Pose2d(11.644, 6.096, Rotation2d.fromDegrees(120))), drivebase))
    // .andThen(Commands.runOnce(() -> new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase,
    // ReefSide.LEFT, ReefIndex.TOP_LEFT, ReefLevel.L4, true).schedule(), drivebase));
    return new SequentialCommandGroup(
      new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase, ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
      new IntakeCommand(wrist, elevator, drivebase, pipeGrabber, new Pose2d(11.644, 6.096, Rotation2d.fromDegrees(120))),
      new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase, ReefSide.LEFT, ReefIndex.TOP_LEFT, ReefLevel.L4, true),
      new IntakeCommand(wrist, elevator, drivebase, pipeGrabber, new Pose2d(11.644, 6.096, Rotation2d.fromDegrees(120)))
    );
  }

  public Command get3PieceTopAuto(){
    Pose2d pickupPose = new Pose2d(16.189, 7.165, Rotation2d.fromDegrees(-125));
    return Commands.runOnce(() -> new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase, 
                              ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, true).schedule(), drivebase)
                              .andThen(Commands.runOnce(() -> new IntakeCommand(wrist, elevator, drivebase, pipeGrabber, pickupPose).schedule(), drivebase))
    .andThen(Commands.runOnce(() -> new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase, 
                              ReefSide.LEFT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true).schedule(), drivebase))
                              .andThen(Commands.runOnce(() -> new IntakeCommand(wrist, elevator, drivebase, pipeGrabber, pickupPose).schedule(), drivebase))
    .andThen(Commands.runOnce(() -> new AutoAlignCommand(wrist, elevator, pipeGrabber, drivebase, 
                              ReefSide.RIGHT, ReefIndex.TOP_RIGHT, ReefLevel.L4, true).schedule(), drivebase));
  }

  public Command getTaxiAuto() {
    return drivebase.getAutonomousCommand("Taxi Auto");
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
  // don't randomly brake/unbrake chassis
  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }
  public void resetHeading() {
    // TODO Auto-generated method stub
    this.drivebase.zeroGyro();
  }
}