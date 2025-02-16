package frc.robot.subsystems.swervedrive;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.awt.Desktop;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

// for using a single limelight instead of the multiple defined in the photonlib vision class
public class Camera {

    // transform for camera relative to robot
    private final Transform3d robotToCamTransform;

    // standard deviations for x, y, theta
    public Matrix<N3, N1> curStdDevs;

    // standard deviations for a single tag
    public Matrix<N3, N1> singleTagStdDevs;

    // estimated robot pose
    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    // Cached results from Limelight
    // public List<LimelightTarget> resultsList = new ArrayList<>();


// Last read timestamp for debounce
    private double lastReadTimestamp = Timer.getFPGATimestamp();

    private final NetworkTable limelightTable;

    public Camera(String name, Rotation3d robotToCamRotation, Translation3d robotToCamTranslation, Matrix<N3, N1> singleTagStdDevs) {
        this.limelightTable = NetworkTableInstance.getDefault().getTable(name);
        robotToCamTransform = new Transform3d(robotToCamTranslation, robotToCamRotation);
        this.curStdDevs = singleTagStdDevs;
    }

    // Get ID of apriltag if present
    public Optional<Integer> getBestTagId() {
        double bestTagId = limelightTable.getEntry("tid").getDouble(-1);  
        double bestAmbiguity = limelightTable.getEntry("t6c_amb").getDouble(-1); 
    
        if (bestTagId == -1 || bestAmbiguity <= 0) { 
            return Optional.empty(); 
        }
    
        return Optional.of((int) bestTagId);
    }

    // same thing as above method but no ambiguity check
    public Optional<Integer> getLatestTagId() {
        double latestId = limelightTable.getEntry("tid").getDouble(-1);
        return (latestId >= 0) ? Optional.of((int) latestId) : Optional.empty();
    }

    // get estimated robot pose
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        updateUnreadResults();
        return estimatedRobotPose;
    }

    private static final double DEBOUNCE_TIME = 0.015; // 15ms

    /**
     * Update the latest target results while maintaining a debounce time of 15ms.
     */
    private void updateUnreadResults() {
        double currentTimestamp = System.nanoTime() / 1.0e9; 
        if (currentTimestamp - lastReadTimestamp < DEBOUNCE_TIME) {
            return; 
        }
        double targetValid = limelightTable.getEntry("tv").getDouble(0); 
        if (targetValid < 1.0) {
            return; 
        }

        // update pose if the target is valid
        lastReadTimestamp = currentTimestamp;
        // updateEstimatedGlobalPose(); 
    }

    /**
     * The latest estimated robot pose on the field from vision data.
     */
    // private void updateEstimatedGlobalPose() {
    //     Optional<EstimatedRobotPose> visionEst = Optional.empty();
    //     for (var change : resultsList) {
    //         visionEst = poseEstimator.update(change);
    //         updateEstimationStdDevs(visionEst, change.getTargets());
    //     }
    //     estimatedRobotPose = visionEst;
    // }

    /**
     * Calculates new standard deviations dynamically for a single Limelight, 
     * based on target visibility and estimated distance.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose) {
    if (estimatedPose.isEmpty()) {
        curStdDevs = singleTagStdDevs;
        return;
    }

    var estStdDevs = singleTagStdDevs;

    double targetArea = limelightTable.getEntry("ta").getDouble(0.0); // target area (distance)
    boolean targetVisible = limelightTable.getEntry("tv").getDouble(0.0) == 1.0; // valid?

    if (!targetVisible || targetArea == 0) {
        curStdDevs = singleTagStdDevs; // no valid tag
    } else {
        // estimate distance using target area
        double estimatedDistance = 1.0 / Math.sqrt(targetArea + 0.01); 
        estimatedDistance = Math.min(estimatedDistance, 10.0); 

        // increase uncertainty for large distances
        if (estimatedDistance > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (estimatedDistance * estimatedDistance / 30));
        }

        curStdDevs = estStdDevs;
    }
}
}
