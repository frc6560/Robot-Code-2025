package frc.robot.commands;

import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Climb.State;
import frc.robot.ManualControls;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {

    private final Climb climb;
    State targetState = State.UP;
    private final ManualControls controls;

    public ClimbCommand(Climb climb, ManualControls controls) {
        this.climb = climb;
        this.controls = controls;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setClimbPosition(ClimbConstants.ClimbStates.DOWN);
    }

    @Override
    public void execute() {
        if (controls.getClimbUp()) {
            targetState = State.UP;

        } else if (controls.getClimbDown()) {
            targetState = State.DOWN;

        }

        if (targetState == State.UP) {
            climb.setClimbPosition(ClimbConstants.ClimbStates.UP);
        } else if (targetState == State.DOWN) {
            climb.setClimbPosition(ClimbConstants.ClimbStates.DOWN);
        } else {}
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false; 
    }
}
