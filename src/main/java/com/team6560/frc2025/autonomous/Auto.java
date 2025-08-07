package com.team6560.frc2025.autonomous;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Defines an Auto class, which is a name, an enum ID, and a command. */
public class Auto {
    private final AutoRoutines autoRoutine;
    private final String name;
    private final Pair<Pose2d, Command> autoCommand;
    private final AutoFactory factory;

    public Auto(AutoRoutines autoRoutine, AutoFactory factory){
        this.autoRoutine = autoRoutine;
        this.factory = factory;

        switch(autoRoutine){
            case IDLE_LEFT:
                this.autoCommand = factory.getNoAutoLeft();
                this.name = "Idle Left";
                break;
            case IDLE_RIGHT:
                this.autoCommand = factory.getNoAutoRight();
                this.name = "Idle Right";
                break;
            // Actually fix these please.
            // case ADAPTIVE:
            //     // do once adaptive autos are done
            //     break;
            // case CENTER_3P:
            //     // do once center piece is done
            //     break;
            // case LEFT_3P:
            //     // do once the left is done
            //     break;
            // case RIGHT_3P:
            //     // do once the right is done
            //     break;
            case LEFT_4P_BACK:
                this.autoCommand = factory.getFourPieceBackLeft();
                this.name = "Left 4P Back";
                break;
            case RIGHT_4P_BACK:
                this.autoCommand = factory.getFourPieceBackRight();
                this.name = "Right 4P Back";
                break;
            case RIGHT_4P:
                this.autoCommand = factory.getFourPieceRight();
                this.name = "Right 4P";
                break;
            case LEFT_4P:
                this.autoCommand = factory.getFourPieceLeft();
                this.name = "Left 4P";
                break;
            default:
                this.autoCommand = Pair.of(new Pose2d(), null);
                this.name = "";
                throw new IllegalArgumentException("Invalid Auto Routine: " + autoRoutine);
        }
    }
}
