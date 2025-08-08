package com.team6560.frc2025;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.OperatorConstants;
import com.team6560.frc2025.commands.BallGrabberCommand;
import com.team6560.frc2025.commands.ClimbCommand;
import com.team6560.frc2025.commands.ElevatorCommand;
import com.team6560.frc2025.commands.PipeGrabberCommand;
import com.team6560.frc2025.commands.WristCommand;
import com.team6560.frc2025.commands.ScoreCommand;
import com.team6560.frc2025.subsystems.BallGrabber;
import com.team6560.frc2025.subsystems.Climb;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.autonomous.Auto;
import com.team6560.frc2025.autonomous.AutoFactory;
import com.team6560.frc2025.autonomous.AutoRoutines;
import com.team6560.frc2025.utility.Enums.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
    pipeGrabberCommand = new PipeGrabberCommand(pipeGrabber, controls);
    pipeGrabber.setDefaultCommand(pipeGrabberCommand);

    wrist = new Wrist();
    wrist.setDefaultCommand(new WristCommand(wrist, controls));
    elevator.setDefaultCommand(new ElevatorCommand(elevator, controls));

    configureBindings();

    factory = new AutoFactory(
      DriverStation.getAlliance().get(),
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

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));

    driverXbox.a().onTrue((Commands.runOnce(drivebase::resetOdometryToLimelight)));

    // Use auto align with scoring
    driverXbox.y().whileTrue(Commands.runOnce(() -> new ScoreCommand(wrist, elevator, pipeGrabber, drivebase,
                              ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L4, false).schedule(), drivebase));
    driverXbox.b().whileTrue(Commands.runOnce(() -> new ScoreCommand(wrist, elevator, pipeGrabber, drivebase,
                              ReefSide.RIGHT, ReefIndex.TOP_LEFT, ReefLevel.L3, false).schedule(), drivebase));

  }

  public void elevL4BeginTele() { // values for auto (don't touch!)
    elevator.setElevatorPosition(17.65);
    wrist.setMotorPosition(40.0);
  }

  public void resetLLBeforeAuto() {
    drivebase.resetOdometryToLimelight();
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
    this.drivebase.zeroGyro();
  }
}