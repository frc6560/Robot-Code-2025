package frc.robot.commands;

import frc.robot.subsystems.PipeGrabber;
import frc.robot.ManualControls;

import edu.wpi.first.wpilibj2.command.Command;
public class PipeGrabberCommand extends Command {
    
    final PipeGrabber grabber;
    final ManualControls controls;

    public PipeGrabberCommand(PipeGrabber grabber, ManualControls controls) {
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
        if (controls.runGrabberIntake()) {
            grabber.runIntake();
        } else if (controls.runOuttake()) {
            grabber.runGrabberOuttake();
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

