package frc.robot.commands;

import frc.robot.subsystems.Climb;
import frc.robot.ManualControls;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;

public class ClimbCommand extends Command {

    private final Climb climb;
    private final ManualControls controls;
    private double test;

    public ClimbCommand(Climb climb, ManualControls controls) {
        this.climb = climb;
        this.controls = controls;
        this.test = 0;
        addRequirements(climb);

        ntDispTab("Climb Debug")
            .add("Climb Test Value", this::getTest);
    }

    public double getTest() {
        return test;
    }

    @Override
    public void initialize() {
        climb.stop();
    }

    @Override
    public void execute() {
        // double speed = -controls.getClimbSpeed(); // Inverted for correct joystick behavior
        // test = speed; // Debugging value to track input
        // climb.setClimbSpeed(); // Apply joystick input to climb
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Keep running until interrupted
    }
}
