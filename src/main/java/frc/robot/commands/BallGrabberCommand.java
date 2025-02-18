package frc.robot.commands;

import frc.robot.subsystems.BallGrabber;
import frc.robot.ManualControls;

import edu.wpi.first.wpilibj2.command.Command;
public class BallGrabberCommand extends Command {
    
    final BallGrabber ballGrabber;
    final ManualControls controls;

    public BallGrabberCommand(BallGrabber grabber, ManualControls controls) {
        this.ballGrabber = grabber;
        this.controls = controls;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        ballGrabber.stop();
    }
    
    @Override
    public void execute() {
        if (controls.pipeGrabberIntake()) {
            ballGrabber.runIntake();
        } else if (controls.pipeGrabberOuttake()) {
            ballGrabber.runOuttake();
        } else {
            ballGrabber.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        ballGrabber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}


