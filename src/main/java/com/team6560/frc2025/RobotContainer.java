package com.team6560.frc2025;

import com.pathplanner.lib.auto.NamedCommands;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.OperatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.commands.BallGrabberCommand;
import com.team6560.frc2025.commands.ClimbCommand;
import com.team6560.frc2025.commands.ElevatorCommand;
import com.team6560.frc2025.commands.PipeGrabberCommand;
import com.team6560.frc2025.commands.WristCommand;
import com.team6560.frc2025.commands.Score;
import com.team6560.frc2025.commands.auto.*;
import com.team6560.frc2025.subsystems.BallGrabber;
import com.team6560.frc2025.subsystems.Climb;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    // CameraServer.startAutomaticCapture(0);

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

    NamedCommands.registerCommand("Scoring L4", new ScoringL4(wrist, elevator, pipeGrabber));
    // NamedCommands.registerCommand("TravelingL3", new L3Travel(wrist, elevator));
    configureBindings();

    autoChooser = new SendableChooser<Command>();

    autoChooser.setDefaultOption("AeroSeg3Inv", getAeroSeg3Inv());


    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
  private void configureBindings() { 

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));

    driverXbox.a().onTrue((Commands.runOnce(drivebase::resetOdometryToLimelight)));

    // Use auto align with scoring
    driverXbox.x().onTrue(new RunCommand(() -> new Score(
      wrist, elevator, pipeGrabber, drivebase, 
      new Pose2d(12.527, 5.227, Rotation2d.fromDegrees(120)), drivebase.getPose(), WristConstants.WristStates.L4 - WristConstants.WristStates.L4Offset
    ).schedule(), drivebase));

    driverXbox.b().whileTrue(new RunCommand(() -> drivebase.driveToNearestPoseRight().schedule(), drivebase));

  }

  public void elevL4BeginTele() { // values for auto (don't touch!)
    elevator.setElevatorPosition(17.65);
    wrist.setMotorPosition(40.0);
  }

  public void resetLLBeforeAuto() {
    drivebase.resetOdometryToLimelight();
  }

  // public Command getAero3PAuto() {
  //   return Commands.parallel(drivebase.getAutonomousCommand("Aero3pSeg1p"), new L4Travel(elevator, wrist))
  //     .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
  //     .andThen(Commands.parallel(drivebase.getAutonomousCommand("Aero3pSeg2p")), new MechanismDown(elevator, wrist))
  //     .andThen(new StationIntake(pipeGrabber, 0.3))
  //     .andThen(Commands.parallel(drivebase.getAutonomousCommand("Aero3pSeg3p"), new StationIntake(pipeGrabber, 1.8)))
  //     .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
  //     .andThen(Commands.parallel(drivebase.getAutonomousCommand("Aero3pSeg4p")), new MechanismDown(elevator, wrist))
  //     .andThen(new StationIntake(pipeGrabber, 0.3))
  //     .andThen(Commands.parallel(drivebase.getAutonomousCommand("Aero3pSeg5p"), new StationIntake(pipeGrabber, 1.8)))
  //     .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
  //     .andThen(drivebase.getAutonomousCommand("Aero3pSeg6p"));
  // }

  public Command getAeroSeg3Inv() {
    return drivebase.getAutonomousCommand("Aero3Part1")
    .alongWith(new L4Travel(elevator, wrist, 2.1))
    .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
    .andThen(drivebase.getAutonomousCommand("Aero3Part2"))
    .andThen((drivebase.getAutonomousCommand("Aero3Part3"))
      .raceWith(new StationIntake(pipeGrabber, 3)))
    .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
    .andThen(drivebase.getAutonomousCommand("Aero3Part4"))
    .andThen((drivebase.getAutonomousCommand("Aero3Part5"))
      .raceWith(new StationIntake(pipeGrabber, 3)))
    .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
    .andThen(drivebase.getAutonomousCommand("Aero3Part6"));
  }

  public Command testElevatorCollapse() {
    return new ScoringL4(wrist, elevator, pipeGrabber)
    .andThen(new WaitCommand(0.5));
  }

  // public Command getAero3pAutoNoProcessor(){
  //   return drivebase.getAutonomousCommand("Aero3p-1")
  //     .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
  //     .andThen(drivebase.getAutonomousCommand("Aero3p-2"))
  //     .andThen(new StationIntake(pipeGrabber, 0.5))
  //     .andThen(drivebase.getAutonomousCommand("Aero3p-3"))
  //     .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
  //     .andThen(drivebase.getAutonomousCommand("Aero3p-4"))
  //     .andThen(new StationIntake(pipeGrabber, 0.5))
  //     .andThen(drivebase.getAutonomousCommand("Aero3p-5"))
  //     .andThen(new ScoringL4(wrist, elevator, pipeGrabber));
  // }

  // public Command getAeroBumpAuto(){
  //   return drivebase.getAutonomousCommand("Aero3pSeg1p")
  //     .andThen(new ScoringL4(wrist, elevator, pipeGrabber))
  //     .andThen(drivebase.getAutonomousCommand("Aero3pSeg2p"))
  //     .andThen(new StationIntake(pipeGrabber, 0.5))
  //     .andThen(drivebase.getAutonomousCommand("Aero3pSeg3p"))
  //     .andThen(drivebase.getAutonomousCommand("AeroBump-4"))
  //     .andThen(drivebase.getAutonomousCommand("AeroBump-5"));
  // }

  // public Command getTestAuto(){
  //   return drivebase.getAutonomousCommand("TestAuto");
  // }

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