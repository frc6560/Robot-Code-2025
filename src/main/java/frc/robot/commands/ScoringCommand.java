// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.State;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.helpers.WristCommand;
import frc.robot.commands.helpers.ElevatorCommand;

public class ScoringCommand extends SequentialCommandGroup {

    public ScoringCommand(Wrist wrist, Elevator elevator, int elevatorTargetState, State wristTargetState) {
        super(
            Commands.sequence(
                new WristCommand(wrist, Wrist.State.STOW),
                new ElevatorCommand(elevator, elevatorTargetState),
                new WristCommand(wrist, wristTargetState)
            )
            .andThen(
                Commands.runOnce(() -> {
                    new WristCommand(wrist, Wrist.State.STOW);
                    new ElevatorCommand(elevator, 0);
                })
            )
        );
    }
}