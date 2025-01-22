package frc.robot.controls;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.Constants;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ManualControls {    

    private XboxController driverController;
    private XboxController interactController;

    private final PovNumberStepper speed;
    private final PovNumberStepper turnSpeed;

    private NetworkTableEntry ntIntakeSpeed;

    /* ManualControls is initialized with two controllers - one for driving, another for shooting (turret control) */
    public ManualControls(XboxController driverController, XboxController interactController) {

        this.driverController = driverController;
        this.interactController = interactController;

        // speed and turnSpeed incremented by POV axes on driver controller

/*         this.speed = new PovNumberStepper(
            new NumberStepper(
                Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controller.SPEED_INITIAL_PERCENT, 
                Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controller.SPEED_MIN_PERCENT,
                Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controller.SPEED_MAX_PERCENT, 
                Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND * Constants.Controller.SPEED_STEP_PERCENT
            ),
            driverController,
            PovNumberStepper.PovDirection.VERTICAL
        );

        this.turnSpeed = new PovNumberStepper(
            new NumberStepper(
                Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controller.TURN_SPEED_INITIAL_PERCENT, 
                Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controller.TURN_SPEED_MIN_PERCENT, 
                Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controller.TURN_SPEED_MAX_PERCENT,
                Constants.Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * Constants.Controller.TURN_SPEED_STEP_PERCENT
            ),
            driverController,
            PovNumberStepper.PovDirection.HORIZONTAL
        );

        ntDispTab("Controls")
        .add("Y Joystick", this::driveY)
        .add("X Joystick", this::driveX)
        .add("Rotation Joystick", this::driveRotationX);

        // Creating a NetworkTableEntry object instead of a NetworkTable allows for retrieving data using .get()

        ntIntakeSpeed = NetworkTableInstance.getDefault().getTable("Intake").getEntry("speed");
        ntIntakeSpeed.setDouble(0.0);
    }
*/
    // UTIL

    /* Deadband - filters out values with absolute value less than deadband, normalizes otherwise. */
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } 
            return (value + deadband) / (1.0 - deadband);
        }
        return 0.0;
    }
    
    /* Filter function applied to raw controller input. Applies deadband and squares value to allow both sensitivity and speed. */
    private static double modifyAxis(double value) {
        value = deadband(value, Constants.Controller.CONTROLLER_DEADBAND);
        value = Math.copySign(value * value, value);
        return value;
    }

    // DRIVING
    // Note: driveX/driveY use reversed controller axis due to 90 degree orientation of robot in relation to field coordinates.
/*
    // Get processed X-Axis driverController left stick input. 
    public double driveX() {
        return - modifyAxis(driverController.getLeftY() * speed.get());
    }

    // Get processed Y-Axis driverController left stick input. 
    public double driveY() {
        return - modifyAxis(driverController.getLeftX() * speed.get());
    }

    // Get processed X-Axis driverController right stick input. 
    public double driveRotationX() {
        return modifyAxis(driverController.getRightX() * turnSpeed.get());
    }

    // Get processed Y-Axis driverController right stick input. 
    public double driveRotationY() {
        return modifyAxis(driverController.getRightY() * turnSpeed.get());
    }
*/
    /* Resets robot yaw (left/right turn) */
    public boolean driveResetYaw() {
        return driverController.getStartButton();
    }
  
    /* Resets robot position */
    public boolean driveResetGlobalPose() {
        return driverController.getBackButton();
    }

    // DRIVETRAIN - SWERVE CALIBRATION

    public boolean setSwerveOffsets() {
        return driverController.getStartButton() && interactController.getStartButton();
    }

    // INTAKE
/*
    // Run intake - A for now 
    public boolean getRunIntake() { 
        return driverController.getAButton();
    }

    // Reverse intake - B for now 
    public boolean getReverseIntake() { 
        return driverController.getBButton();
    }

    // FEEDER

    // Run feeder (fire) - Right shoulder button 
    
    public boolean getRunFeeder() {
        return driverController.getRightBumper();
    }
*/
    // ELEVATOR

    /* Run elevator - X for now */
    public boolean getRunShooter() { 
        return driverController.getLeftBumper();
    }

    /* Reverse shooter - Y for now */
    public boolean getReverseShooter() { 
        return driverController.getYButton();
    }

    // TURRET

    /* Bring turret up */
    public boolean getTurretUp() { 
        return interactController.getRightBumper();
    }

    /* Bring turret down */
    public boolean getTurretDown() { 
        return interactController.getLeftBumper();
    }

    /* Turn turret clockwise */
    public boolean getTurretClockwise() { 
        return interactController.getAButton();
    }

    /* Turn turret counterclockwise */
    public boolean getTurretCounterClockwise() { 
        return interactController.getBButton();
    }

    // RUMBLE

    /* Set rumble level of driver controller. Accepts values from 0 to 1 */
    public void setDriverControllerRumble(double output){
        driverController.setRumble(RumbleType.kBothRumble, output);
    }
    
    /* Set rumble level of shooter controller. Accepts values from 0 to 1 */
    public void setinteractControllerRumble(double output){
        interactController.setRumble(RumbleType.kBothRumble, output);
    }
}