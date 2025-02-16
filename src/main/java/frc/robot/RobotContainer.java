package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.OperatorConstants;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Limelight;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.ClimbCommand;
import frc.robot.commands.GrabberCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.ElevatorCommand;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

  final CommandXboxController driverXbox = new CommandXboxController(0);
  final XboxController firstXbox = new XboxController(0);
  final XboxController secondXbox = new XboxController(1);

  private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/falcon"));

  private final Climb climb;
  private final ClimbCommand climbCommand;
  private final Grabber grabber;
  private final GrabberCommand grabberCommand;

  private final Wrist wrist;
  private final Elevator elevator;

  // private final Limelight limelight;

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
    .withControllerRotationAxis(driverXbox::getRightX)
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  public RobotContainer() {
    climb = new Climb(controls);
    climbCommand = new ClimbCommand(climb, controls);
    // climb.setDefaultCommand(climbCommand);

    grabber = new Grabber();
    grabberCommand = new GrabberCommand(grabber, controls);
    grabber.setDefaultCommand(grabberCommand);

    wrist = new Wrist();
    wrist.setDefaultCommand(new WristCommand(wrist, controls));

    elevator = new Elevator();
    elevator.setDefaultCommand(new ElevatorCommand(elevator, controls));

    // limelight = new Limelight();

    configureBindings();
  }
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
  }
  
  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("Test Auto");
  }
  
  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
