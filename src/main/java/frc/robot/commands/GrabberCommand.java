package frc.robot.commands;

import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;
import frc.robot.ManualControls;

import edu.wpi.first.wpilibj2.command.Command;
public class GrabberCommand extends Command {
    
    final Grabber grabber;
    final ManualControls controls;

    public GrabberCommand(Grabber grabber, ManualControls controls) {
        this.grabber = grabber;
        this.controls = controls;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        grabber.stop();
    }
    
    @Override
    public void execute() {
        if (controls.runIntake()) {
            grabber.runIntake();
        } else {
            grabber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

