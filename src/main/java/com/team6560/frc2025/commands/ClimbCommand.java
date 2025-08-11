package com.team6560.frc2025.commands;

import com.team6560.frc2025.controls.XboxControls;
import com.team6560.frc2025.subsystems.Climb;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {

    private final Climb climb;
    // State targetState = State.UP;
    private final XboxControls controls;

    public ClimbCommand(Climb climb, XboxControls controls) {
        this.climb = climb;
        this.controls = controls;

        addRequirements(climb);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (controls.getClimbDown()) {
            // climb.state = Climb.State.DOWN;
            climb.down();
        } else if (controls.getClimbUp()) {
            // climb.state = Climb.State.UP;
            climb.up();
        } else {
             // climb.state = Climb.State.STATIC; continuous motion
            climb.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
