package frc.robot.commands.auto;

import frc.robot.subsystems.Grabber;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;


public class Scoring extends Command{
    private Grabber grabber;
    private boolean finished = false;
    private Timer grabberTimer;

    public Scoring(Grabber grabber){
        this.grabber = grabber;
    }

    @Override
    public void initialize(){
        grabber.stop();
        this.grabberTimer = new Timer();
    }

    @Override
    public void execute(){
        grabberTimer.start();
        if(grabberTimer.hasElapsed(1.0)){
            finished = true;
        }
        grabber.runOuttake();
    }

    @Override
    public void end(boolean interrupted){
        grabber.stop();
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}