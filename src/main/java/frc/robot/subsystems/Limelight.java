package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.utility.NetworkTable.NtValueDisplay.ntDispTab;


import java.util.Arrays;
import java.util.function.Supplier;

import frc.robot.Constants;

public class Limelight {
    public static interface Controls {
        int getLimelightPipeline();
    }

    private final NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("Limelight");

    private final NetworkTableEntry ntX = networkTable.getEntry("tx");
    private final NetworkTableEntry ntY = networkTable.getEntry("ty");
    private final NetworkTableEntry ntV = networkTable.getEntry("tv");
    private final NetworkTableEntry ntA = networkTable.getEntry("ta"); // values
    private final NetworkTableEntry ntL = networkTable.getEntry("tl"); // latency
    private final NetworkTableEntry ntcL = networkTable.getEntry("cl");
    private final NetworkTableEntry ntBotPose = networkTable.getEntry("botpose_wpiblue");
    private final NetworkTableEntry ntPipeline = networkTable.getEntry("pipeline");

    private final Field2d aprilTagField = new Field2d();

    private final Controls controls;
    private boolean forceOff = true;

    private Supplier<Pose2d> predictedPose;

    /** Creates a new Limelight object */
    public Limelight(Controls controls, Supplier<Pose2d> predictedPose) {
        this.controls = controls;
        this.predictedPose = predictedPose;

        setForceOff(false);

        ntDispTab("Limelight")
            .add("Horizontal Angle", this::getHorizontalAngle)
            .add("Vertical Angle", this::getVerticalAngle)
            .add("Has Target", this::hasTarget);

        SmartDashboard.putData("aprilTagField", aprilTagField);
    }

    // Obtains the values from Limelight's network table entry.
    /** Obtains pipeline */
    public int getPipeline(){
        return controls.getLimelightPipeline();
    }

    /** Obtains horizontal angle */
    public double getHorizontalAngle(){
        return ntX.getDouble(0);
    }

    /** Obtains vertical angle */
    public double getVerticalAngle(){
        return ntY.getDouble(0);
    }

    /** Obtains target area */
    public double getTargetArea(){
        return ntA.getDouble(0);
    }

    /** Obtains whether the target is in view */
    public boolean hasTarget(){
        return ntV.getDouble(0) == 1;
    }

    /** Toggles force off */ 
    void setForceOff(boolean forceOff) {
        this.forceOff = forceOff;
    }

    /** Returns a Pose3D representation of the robot heading */
    public Pose3d getTargetBotPose3D(){
        double[] limelightBotPoseArray = networkTable.getEntry("targetpose_robotspace").getDoubleArray(new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        
        if (limelightBotPoseArray == null || limelightBotPoseArray.length < 6) return null;

        if (new double[] {0.0, 0.0, 0.0, 0.0, 0.0}.equals(Arrays.copyOf(limelightBotPoseArray, limelightBotPoseArray.length - 1)))
            return null;
    
        return new Pose3d(new Translation3d(limelightBotPoseArray[0], limelightBotPoseArray[1], limelightBotPoseArray[2]), new Rotation3d(Math.toRadians(limelightBotPoseArray[3]), Math.toRadians(limelightBotPoseArray[4]), Math.toRadians(limelightBotPoseArray[5])));
    }

    /** Gets the April Tag ID */
    public int getAprilTagID(){
        return (int) networkTable.getEntry("tid").getInteger(0l);
    }

    
}
