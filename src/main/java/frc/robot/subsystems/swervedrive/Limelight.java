// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import frc.robot.subsystems.swervedrive.Camera;

public class Limelight extends SubsystemBase {

  // field layout and pose suppliers
  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  public VisionSystemSim visionSim;
  private Supplier<Pose2d> currentPose;
  private Field2d field2d;

  private Camera camera;
    
  // initialize limelight with Field2d and pose
  public Limelight(Supplier<Pose2d> currentPose, Field2d field) {

    this.currentPose = currentPose;
    this.field2d = field;

    // this is the actual limelight object - remove if unnecessary, original written for multiple cameras
    this.camera = new Camera(
        "test", 
        new Rotation3d(0, 0, 0), // No rotation
        new Translation3d(0, 0, 0), // No translation
        VecBuilder.fill(0.1, 0.1, 0.1) // default stdevs for tag
    );
  }

  // get 2d pose of apriltag on the field given an index
  public static Pose2d getAprilTagPose(int aprilTag, Transform2d robotOffset) {
    Optional<Pose3d> aprilTagPose3d = fieldLayout.getTagPose(aprilTag);
    if (aprilTagPose3d.isPresent()) {
        return aprilTagPose3d.get().toPose2d().transformBy(robotOffset);   
    } else {
        throw new RuntimeException("Cannot get AprilTag " + aprilTag + " from field " + fieldLayout.toString());
    }
  }

  // update pose estimation in SwerveDrive using given poses
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    Optional<EstimatedRobotPose> poseEst = getEstimatedGlobalPose(camera);
    if (poseEst.isPresent()) {
        var pose = poseEst.get();
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camera.curStdDevs);
    }
  }

  // this requires 'Cameras' as input
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {
    return camera.getEstimatedGlobalPose();
  }

  // get distance of robot rom apriltag pose
  // public double getDistanceFromAprilTag(int id) {
  //   Optional<Pose3d> tag = fieldLayout.getTagPose(id);
  //   // replace photonPose method with limelight logic
  // }

  @Override
  public void periodic() {
    // ntPipeline.setNumber(forceOff ? 0 : controls.getLimelightPipeline());
  }
}