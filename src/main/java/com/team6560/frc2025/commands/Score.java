package com.team6560.frc2025.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.team6560.frc2025.ManualControls;
import com.team6560.frc2025.Constants.ElevatorConstants;
import com.team6560.frc2025.Constants.WristConstants;
import com.team6560.frc2025.subsystems.Elevator;
import com.team6560.frc2025.subsystems.PipeGrabber;
import com.team6560.frc2025.subsystems.Wrist;
import com.team6560.frc2025.subsystems.swervedrive.SwerveSubsystem;
import com.team6560.frc2025.utility.AutoAlignPath;
import com.team6560.frc2025.utility.Setpoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class Score extends SequentialCommandGroup {
    public boolean canDunk = false;

    public Score(Wrist wrist, Elevator elevator, PipeGrabber grabber, SwerveSubsystem drivetrain, 
                    Pose2d targetPose, Pose2d currentPose, double targetWrist) {

        Timer grabTimer = new Timer();
        Timer downTimer = new Timer();

        double wristDunkAngle = 6.56; 
        double elevatorTarget = ElevatorConstants.ElevatorStates.L4;

        double E_TOLERANCE = 1.0;
        double W_TOLERANCE = 8.0;

        AutoAlignPath path = new AutoAlignPath(
                            currentPose,
                            targetPose,
                            1.5, 
                            1.0, 
                            3.14, 
                            1.57); 

        TrapezoidProfile.State translationalState = new TrapezoidProfile.State(0, 0);
        TrapezoidProfile.State rotationalState = new TrapezoidProfile.State(0, 0);

        TrapezoidProfile.State targetTranslationalState = new TrapezoidProfile.State(0, 0); // The position is actually the error.
        TrapezoidProfile.State targetRotationalState = new TrapezoidProfile.State(0, 0);

        // Set up profiles
        TrapezoidProfile.Constraints translationConstraints = new Constraints(path.maxVelocity, path.maxAcceleration);
        TrapezoidProfile.Constraints rotationConstraints = new Constraints(path.maxAngularVelocity,path.maxAngularAcceleration);
        TrapezoidProfile translationProfile = new TrapezoidProfile(translationConstraints);
        TrapezoidProfile rotationProfile = new TrapezoidProfile(rotationConstraints);

        final Command mechanismUp = new FunctionalCommand(
                    () -> {
                        // Compute current relative states for translation and rotation, with (0, 0) being target for translation.
                        translationalState.position = path.getDisplacement().getNorm();
                        translationalState.velocity = MathUtil.clamp(Math.sqrt(Math.pow(drivetrain.getFieldVelocity().vxMetersPerSecond, 2) + Math.pow(drivetrain.getFieldVelocity().vyMetersPerSecond, 2)),
                                                    -path.maxVelocity,
                                                    0);

        
                        targetRotationalState.position = path.endPose.getRotation().getRadians();
                        rotationalState.position = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
                        rotationalState.velocity = drivetrain.getSwerveDrive().getRobotVelocity().omegaRadiansPerSecond;
                    },
                    () -> {
                        // // Sets subsystems.
                        elevator.setElevatorPosition(elevatorTarget);

                        // Compute the next translation state for the robot to target (t+0.02s).
                        State translationSetpoint = translationProfile.calculate(0.02, translationalState, targetTranslationalState);
                        translationalState.position = translationSetpoint.position;
                        translationalState.velocity = translationSetpoint.velocity;

                        // no mod 360 shenanigans
                        double rotationalPose = drivetrain.getSwerveDrive().getPose().getRotation().getRadians();
                        double errorToGoal = MathUtil.angleModulus(targetRotationalState.position - rotationalPose);
                        double errorToSetpoint = MathUtil.angleModulus(rotationalState.position - rotationalPose);

                        // feeds target (and current) poses without mod 360 shenanigans. also for feedback :)
                        targetRotationalState.position = rotationalPose + errorToGoal;
                        rotationalState.position = rotationalPose + errorToSetpoint;

                        // Computes the next rotation state for the robot to target.
                        State rotationalSetpoint = rotationProfile.calculate(0.02, rotationalState, targetRotationalState);
                        rotationalState.position = rotationalSetpoint.position;
                        rotationalState.velocity = rotationalSetpoint.velocity;

                        // converts our state to a field relative pose to target.
                        Translation2d interpolatedTranslation = path.endPose
                            .getTranslation()
                            .interpolate(path.startPose.getTranslation(), 
                            translationSetpoint.position / path.getDisplacement().getNorm());
                        
                        double targetWristAngle = WristConstants.WristStates.L4;


                        if((translationalState.position < 0.1) && (Math.abs(rotationalState.position - targetRotationalState.position) < 0.1)){
                            drivetrain.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
                            if(Math.abs(elevator.getElevatorHeight() - elevatorTarget) < E_TOLERANCE && 
                            Math.abs(wrist.getWristAngle()) - 195 < W_TOLERANCE){ // TODO: no magic numbers
                                canDunk = true;
                            }
                            if(canDunk){
                                targetWristAngle = wristDunkAngle;
                                System.out.println(wrist.getWristAngle());
                                if(Math.abs(wrist.getWristAngle() - (-233)) < W_TOLERANCE){ // TODO: remove magic numbers :D
                                    grabTimer.start();
                                    grabber.runGrabberOuttakeMaxSpeed();
                                }
                            }
                        }
                        else {
                            drivetrain.followSegment(new Setpoint(
                                interpolatedTranslation.getX(),
                                interpolatedTranslation.getY(),
                                rotationalState.position,
                                path.getNormalizedDisplacement().getX() * -translationSetpoint.velocity, 
                                path.getNormalizedDisplacement().getY() * -translationSetpoint.velocity,
                                rotationalState.velocity
                            ));
                        }

                    wrist.setMotorPosition(targetWristAngle);
                    },
                    (interrupted) -> {},
                    () -> grabTimer.hasElapsed(0.25) // TODO: replace with something actual
        );

        final Command mechanismDown = new FunctionalCommand(
                        () -> {
                            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.L4);
                            wrist.setMotorPosition(WristConstants.WristStates.STOW);
                            downTimer.reset();
                            downTimer.start();
                        },
                        () -> {
                            wrist.setMotorPosition(WristConstants.WristStates.STOW);
                            if(downTimer.hasElapsed(0.6)){ // TOOD: NO TIMERS >:(
                                elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                            }
                            elevator.setElevatorPosition(ElevatorConstants.ElevatorStates.STOW);
                        },
                        (interrupted) -> {},
                        () -> (Math.abs(elevator.getElevatorHeight() - ElevatorConstants.ElevatorStates.STOW) < 1.0)
        );

        super.addCommands(mechanismUp, mechanismDown);
        super.addRequirements(wrist, elevator, grabber, drivetrain);
    }

}
