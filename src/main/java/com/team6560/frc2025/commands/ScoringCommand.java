// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.
package com.team6560.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.Wrist.State;
import com.team6560.frc2025.subsystems.Elevator;

import com.team6560.frc2025.commands.helpers.WristCommand;
import com.team6560.frc2025.commands.helpers.ElevatorCommand;

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