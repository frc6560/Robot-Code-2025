package com.team6560.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2025.subsystems.BallGrabber;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;
import com.team6560.frc2025.utility.Setpoint;
import com.team6560.frc2025.Constants.REEF_MEASUREMENTS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import com.team6560.frc2025.commands.BargeCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;

public class BaregCommandHelpers {

    public static Boolean isIntersecting(Pose2d start, Pose2d end) {
        Translation2d startpos = start.getTranslation();
        Translation2d endpos = end.getTranslation();

        double distance = startpos.getDistance(endpos);
        return distance < 0.2;

    }
}
