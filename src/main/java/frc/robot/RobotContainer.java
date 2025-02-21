package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.PipeGrabber;
// import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;

// import frc.robot.commands.ClimbCommand;
import frc.robot.commands.PipeGrabberCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.auto.*;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final XboxController firstXbox = new XboxController(0);
  final XboxController secondXbox = new XboxController(1);

  private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

  // private final Climb climb;
  // private final ClimbCommand climbCommand;
  private final PipeGrabber grabber;
  private final PipeGrabberCommand grabberCommand;

  private final Wrist wrist;
  private final Elevator elevator;

  private final SendableChooser<Command> autoChooser;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
    .withControllerRotationAxis(driverXbox::getRightX)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  public RobotContainer() {

    // climb = new Climb(controls);
    // climbCommand = new ClimbCommand(climb, controls);
    // climb.setDefaultCommand(climbCommand);

    grabber = new PipeGrabber();
    grabberCommand = new PipeGrabberCommand(grabber, controls);
    grabber.setDefaultCommand(grabberCommand);

    wrist = new Wrist();
    wrist.setDefaultCommand(new WristCommand(wrist, controls));

    elevator = new Elevator();
    elevator.setDefaultCommand(new ElevatorCommand(elevator, controls));

    NamedCommands.registerCommand("l3", new L3(wrist, elevator));
    NamedCommands.registerCommand("score", new Scoring(grabber));
    NamedCommands.registerCommand("dunk", new Dunk(wrist));
    NamedCommands.registerCommand("stow", new Stow(wrist, elevator));
    configureBindings();

    autoChooser = new SendableChooser<Command>();
    autoChooser.addOption("Turning Simple", turningSimple());
    autoChooser.addOption("No Auto", null);
    autoChooser.addOption("Subsystems Test Auto", subsystemsTest());
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // configure drivetrain bullshit - do this with a normal XboxController if easier
  private void configureBindings() {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.b().whileTrue(
    //   drivebase.driveToPose(
    //     new Pose2d(new Translatio n2d(4, 4), Rotation2d.fromDegrees(0))));
    
    //  driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  public Command turningSimple() {
    return drivebase.getAutonomousCommand("Subsystems Test Auto");
  }

  public Command subsystemsTest(){
    return drivebase.getAutonomousCommand("Subsystems Test Auto");
  }
  
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    return drivebase.getAutonomousCommand("Subsystems Test Auto");
  }
  
  public void setMotorBrake(boolean brake) {
    // drivebase.setMotorBrake(brake);
  }
}