package com.team6560.frc2025.commands.auto;

import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
public class AutoAlignLeft extends Command {

    private SwerveSubsystem drivebase;

    public AutoAlignLeft(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
    }

    // I did not test this!!! And we might not actually need it

    @Override
    public void initialize() {
        Commands.runOnce(drivebase::resetOdometryToLimelight);
    }

    @Override
    public void execute() {
        drivebase.driveToNearestPoseLeft();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
