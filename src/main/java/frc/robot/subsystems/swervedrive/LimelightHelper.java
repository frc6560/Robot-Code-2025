package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.Pair;

// this is just whatever was in 2023 without unnecessary stuff
public class LimelightHelper {

    private static final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    private static final NetworkTableEntry ntBotPose = networkTable.getEntry("botpose_wpiblue"); // Change for red alliance
    private static final NetworkTableEntry ntV = networkTable.getEntry("tv"); // Valid target?
    private static final NetworkTableEntry ntPipeline = networkTable.getEntry("pipeline"); // Check active pipeline
    private static final NetworkTableEntry ntL = networkTable.getEntry("tl"); // latency

    public static Pair<Pose2d, Double> getLimelightPose() {

        // give latency feedback
        double latencySeconds = (ntL.getDouble(0.0) + 11) / 1000.0; // 11ms recommended capture latency
        double timestamp = Timer.getFPGATimestamp() - latencySeconds;

        Pair<Pose2d, Double> emptyPose = new Pair<>(new Pose2d(), timestamp);

        // make sure pipeline is correct
        if (ntPipeline.getInteger(0) != 0 && ntPipeline.getInteger(0) != 1) {
            return emptyPose;
        }
        
        if (ntV.getDouble(0.0) == 0.0) {
            return emptyPose;
        }

        // get bot pose
        double[] botPose = ntBotPose.getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

        // validate array
        if (botPose.length < 6) {
            return emptyPose;
        } 

        // extract coordinates
        double xMeters = botPose[0];  
        double yMeters = botPose[1];  
        double rotationDegrees = botPose[5];

        // create pose
        Pose2d pose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(rotationDegrees));

        return new Pair<>(pose, timestamp);
    }
}
