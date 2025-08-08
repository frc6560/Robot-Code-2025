package com.team6560.frc2025.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** Defines an Auto class, which is a name, an enum ID, and a command. */
public class Auto {
    private final AutoRoutines autoRoutine;
    private final String name;
    private Pair<Pose2d, Command> autoCommand;
    private final AutoFactory factory;

    public Auto(AutoRoutines autoRoutine, AutoFactory factory){
        this.autoRoutine = autoRoutine;
        this.factory = factory;
        this.autoCommand = null;
        this.name = getAutos().getSecond();
        
    }

    public Command getCommand() {
        update();
        return autoCommand.getSecond();
    }

    public Pose2d getStartPose(){
        update();
        return autoCommand.getFirst();
    }

    public String getName() {
        return name;
    }

    public void update(){
        this.factory.updateAlliance(DriverStation.getAlliance().get());
        Pair<Pair<Pose2d, Command>, String> autos = getAutos();
        this.autoCommand = autos.getFirst();
    }

    public Pair<Pair<Pose2d, Command>, String> getAutos(){
        Pair<Pose2d, Command> command;
        String name;
        switch(autoRoutine){
            case IDLE_LEFT:
                command = factory.getNoAutoLeft();
                name = "Idle Left";
                break;
            case IDLE_RIGHT:
                command = factory.getNoAutoRight();
                name = "Idle Right";
                break;
            case CENTER_3P:
                command = factory.getThreePieceCenter();
                name = "Center 3P";
                break;
            case LEFT_3P:
                command = factory.getThreePieceBackLeft();
                name = "Left 3P";
                break;
            case RIGHT_3P:
                command = factory.getThreePieceBackRight();
                name = "Right 3P";
                break;
            case LEFT_4P_BACK:
                command = factory.getFourPieceBackLeft();
                name = "Left 4P Back";
                break;
            case RIGHT_4P_BACK:
                command = factory.getFourPieceBackRight();
                name = "Right 4P Back";
                break;
            case RIGHT_4P:
                command = factory.getFourPieceRight();
                name = "Right 4P";
                break;
            case LEFT_4P:
                command = factory.getFourPieceLeft();
                name = "Left 4P";
                break;
            case TEST:
                command = factory.getTest();
                name = "Test Auto";
                break;
            default:
                command = Pair.of(new Pose2d(), null);
                name = "";
                throw new IllegalArgumentException("Invalid Auto Routine: " + autoRoutine);
        }
        return Pair.of(command, name);
    }
}
