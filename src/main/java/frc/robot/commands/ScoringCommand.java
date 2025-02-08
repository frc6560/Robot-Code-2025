/**
 * The `ScoringCommand` class is a sequential command group that orchestrates a series of commands to
 * control the wrist and elevator subsystems for scoring in a robotics project.
 */
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;
import frc.robot.subsystems.Elevator;
import frc.robot.ManualControls;

import frc.robot.commands.helpers.WristCommand;
import frc.robot.commands.helpers.ElevatorCommand;

public class ScoringCommand extends Command{
    private final Wrist wrist;
    private final Elevator elevator;
    final ManualControls controls;

    public ScoringCommand(Wrist wrist, Elevator elevator, ManualControls controls) {
        this.wrist = wrist;
        this.elevator = elevator;

        this.controls = controls;
        addRequirements(wrist, elevator);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
        if(controls.getRunScoreL2()){
            new WristCommand(wrist, State.L2).schedule();
            new ElevatorCommand(elevator, 2).schedule();
        } else if(controls.getRunScoreL3()){
            new WristCommand(wrist, State.L2).schedule();
            new ElevatorCommand(elevator, 3).schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        new WristCommand(wrist, Wrist.State.STOW).schedule();
        new ElevatorCommand(elevator, 0).schedule();
    }

    @Override
    public boolean isFinished() {
        // This command should run continuously until all buttons are released
        return !controls.getRunScoreL2() && !controls.getRunScoreL3();
    }
}