package com.team6560.frc2025.commands;

import com.team6560.frc2025.controls.ButtonBoard;
import com.team6560.frc2025.controls.XboxControls;
import com.team6560.frc2025.subsystems.PipeGrabber;

import edu.wpi.first.wpilibj2.command.Command;
public class PipeGrabberCommand extends Command {
    
    final PipeGrabber grabber;
    final XboxControls controls;
    final ButtonBoard board;

    public PipeGrabberCommand(PipeGrabber grabber, XboxControls controls, ButtonBoard board) {
        this.grabber = grabber;
        this.controls = controls;
        this.board = board;
        addRequirements(grabber);
    }

    @Override
    public void initialize() {
        grabber.stop();
    }
    
    @Override
    public void execute() {
            if (controls.runGrabberOuttake()) {
                if(!board.getL1()){
                    grabber.runGrabberOuttake();
                }
                else {
                    grabber.runGrabberOuttakeL1();
                }
            } else{
                grabber.stop();
            }
            if(board.getIntake()){
                grabber.runIntake();
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

