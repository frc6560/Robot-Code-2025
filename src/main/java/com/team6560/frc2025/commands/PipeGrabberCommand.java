package com.team6560.frc2025.commands;

import com.team6560.frc2025.ManualControls;
import com.team6560.frc2025.subsystems.PipeGrabber;

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
        if (!controls.shiftedControls()) {
            if (controls.runGrabberIntake()) {
                grabber.runIntake();
            } else if (controls.runGrabberOuttake()) {
                grabber.runGrabberOuttake();
            } else {
                grabber.stop();
            }
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

