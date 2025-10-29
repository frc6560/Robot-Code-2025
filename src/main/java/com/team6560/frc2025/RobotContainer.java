package com.team6560.frc2025;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.OperatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.commands.BallGrabberCommand;
import com.team6560.frc2025.commands.ClimbCommand;
import com.team6560.frc2025.commands.ElevatorCommand;
import com.team6560.frc2025.commands.PipeGrabberCommand;
import com.team6560.frc2025.commands.WristCommand;
import com.team6560.frc2025.commands.automations.CoralScoreCommandFactory;
import com.team6560.frc2025.controls.ButtonBoard;
import com.team6560.frc2025.controls.XboxControls;
import com.team6560.frc2025.subsystems.BallGrabber;
import com.team6560.frc2025.subsystems.Climb;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.Enums.ReefLevel;
import com.team6560.frc2025.subsystems.LocationManager;
import com.team6560.frc2025.autonomous.Auto;
import com.team6560.frc2025.autonomous.AutoFactory;
import com.team6560.frc2025.autonomous.AutoRoutines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.util.Set;

import com.frc3481.swervelib.SwerveInputStream;

// took out subsystems + added camera
public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final XboxController firstXbox = new XboxController(0);
  final XboxController secondXbox = new XboxController(1);
  final ButtonBoard buttonBoard = new ButtonBoard(1, 2); 

  private final XboxControls controls = new XboxControls(firstXbox, secondXbox);
  private final LocationManager locationManager = new LocationManager(buttonBoard);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

  private final Climb climb;
  private final ClimbCommand climbCommand;
  private final PipeGrabber pipeGrabber;
  private final PipeGrabberCommand pipeGrabberCommand;
  private final BallGrabber ballGrabber;
  private final BallGrabberCommand ballGrabberCommand;
  private final Wrist wrist;
  private final Elevator elevator = new Elevator();

  private final CoralScoreCommandFactory scoreFactory;

  private final SendableChooser<Auto> autoChooser;
  private final AutoFactory factory;

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
    pipeGrabberCommand = new PipeGrabberCommand(pipeGrabber, controls, buttonBoard);
    pipeGrabber.setDefaultCommand(pipeGrabberCommand);

    wrist = new Wrist();
    wrist.setDefaultCommand(new WristCommand(wrist, controls));
    elevator.setDefaultCommand(new ElevatorCommand(elevator, controls));

    scoreFactory = new CoralScoreCommandFactory(wrist, elevator, pipeGrabber, drivebase, ballGrabber);

    configureBindings();

    factory = new AutoFactory(
      null,
      wrist,
      elevator,
      drivebase,
      pipeGrabber
    );

    autoChooser = new SendableChooser<Auto>();

    for(AutoRoutines auto : AutoRoutines.values()) {
      Auto autonomousRoutine = new Auto(auto, factory);
      if(auto == AutoRoutines.TEST){
        autoChooser.setDefaultOption(autonomousRoutine.getName(), autonomousRoutine);
      }
      else {
        autoChooser.addOption(autonomousRoutine.getName(), autonomousRoutine);
      }
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() { 
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    // Trigger for auto align
    Trigger autoAlignTrigger = new Trigger(
      () -> (locationManager.hasTarget() && locationManager.inReefMode())
    );

    Trigger ballEjectTrigger = new Trigger(
      () -> (buttonBoard.getIntake() && !locationManager.inReefMode())
    );

    Trigger l1Trigger = new Trigger(
      () -> buttonBoard.getL1() && locationManager.inReefMode()
    );

    Trigger l2BallTrigger = new Trigger(
      () -> buttonBoard.getL2() && buttonBoard.getShift()
    );

    Trigger l3BallTrigger = new Trigger(
      () -> buttonBoard.getL3() && buttonBoard.getShift()
    );

    Trigger l4BallTrigger = new Trigger(
      () -> buttonBoard.getL4() && buttonBoard.getShift()
    );

    autoAlignTrigger.onTrue(Commands.defer(
      () -> {
        if(!scoreFactory.isTagVisible(locationManager.getReefSide())){
          System.out.println("Failed to auto align");
          return Commands.none();
        }
        return scoreFactory.getScoreTeleop(locationManager.getCurrentReefLevel(), locationManager.getReefSide());
      }, Set.of(drivebase, wrist, elevator, pipeGrabber)).finallyDo((interrupted) -> locationManager.reset())
    );

    ballEjectTrigger.whileTrue(Commands.defer(
      () -> Commands.either(
        new RunCommand(() -> ballGrabber.runOuttake(), ballGrabber),
        new RunCommand(() -> ballGrabber.runIntake(), ballGrabber),
        () -> (locationManager.getCurrentReefLevel() == null || 
                locationManager.getCurrentReefLevel() == ReefLevel.L4))
        , Set.of(ballGrabber)).finallyDo((interrupted) -> {
          ballGrabber.stop();
        }));

    l1Trigger.whileTrue(Commands.run(() -> wrist.setMotorPosition(WristConstants.WristStates.L1)));
    l2BallTrigger.whileTrue(Commands.run(() -> {
                                        wrist.setMotorPosition(WristConstants.WristStates.S_L2);
                                        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.S_L2);
                                      }));
    l3BallTrigger.whileTrue(Commands.run(() -> {
                                        wrist.setMotorPosition(WristConstants.WristStates.S_L2);
                                        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.S_L3);
                                      }));
    l4BallTrigger.whileTrue(Commands.run(() -> {
                                        wrist.setMotorPosition(WristConstants.WristStates.S_L4);
                                        elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.S_L4);
                                      }));
    
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
    driverXbox.a().onTrue((Commands.runOnce(() -> drivebase.updateOdometryWithVision("limelight-right"))));
    driverXbox.y().onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));
  }

  public void elevL4BeginTele() { // values for auto (don't touch!)
    elevator.setElevatorPosition(17.65);
    wrist.setMotorPosition(40.0);
  }

  public void resetLLBeforeAuto() {
  }


  public Auto getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  
  // don't randomly brake/unbrake chassis
  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }
  public void resetHeading() {
    // TODO Auto-generated method stub
    drivebase.resetOdometry(new Pose2d(drivebase.getPose().getTranslation(), Rotation2d.fromDegrees(0)));
    drivebase.zeroGyro();
  }

  /** Subsystem getters */
  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public Climb getClimb() {
    return climb;
  }

  public PipeGrabber getPipeGrabber() {
    return pipeGrabber;
  }

  public BallGrabber getBallGrabber() {
    return ballGrabber;
  }

  public Wrist getWrist() {
    return wrist;
  }

  public Elevator getElevator() {
    return elevator;
  }

  public XboxControls getControls() {
    return controls;
  }

  public ButtonBoard getButtonBoard() {
    return buttonBoard;
  }

  public LocationManager getLocationManager() {
    return locationManager;
  }
}