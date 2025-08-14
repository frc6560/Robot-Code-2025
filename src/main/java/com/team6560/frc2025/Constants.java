package com.team6560.frc2025;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import com.frc3481.swervelib.math.Matter;
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);

  public static final double WHEEL_COF = 1.19; // subject to much change.

  public static final double ROBOT_WIDTH = 0.965;
  public static final double ROBOT_LENGTH = 0.838;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    
  }

  public static final class ElevatorConstants {

    public static int ELEV_LEFT_ID = 14;
    public static int ELEV_RIGHT_ID = 15;
    public static int ELEV_UPPER_LIMIT_SWITCH_ID = 28;
    public static int ELEV_LOWER_LIMIT_SWITCH_ID = 29;

    public static double ELEV_GEAR_RATIO = 2.66;
    public static final class ElevatorStates {

      public static final double STOW = 0.4;
      public static final double L2 = 0.605; 
      public static final double L3 = 6.62;
      public static final double L4 = 18.03; 

      public static final double S_STOW = 0.4;
      public static final double S_L2 = 1.51;
      public static final double S_L3 = 8.15;  
      public static final double S_L4 = 20.8;
    }
  }

  public static final class WristConstants {

    public static final int M_ID = 16;
    public static final int SWITCH_ID = 22;  
    public static final int CANCODER_ID = 17; 

    public static final double GEAR_RATIO = 108;

    public static final double UPPER_SOFT_BOUND = 254.0; 
    public static final double LOWER_SOFT_BOUND = -5.0;

    public static final int LIMIT_SWITCH_PORT_ID = 19;
    public static final class WristStates {
      public static final double STOW = 90.0;
      public static final double PICKUP = 225;  
      public static final double L1 = 200; 
      public static final double L2 = 60; 
      public static final double L4 = 34; 

      public static final double S_STOW = 208.5; 
      public static final double S_L2 = 148.5; 
      public static final double S_L4 = 250; 

      public static final double L1Offset = 0.0;
      public static final double L2Offset = 35.0;
      public static final double L4Offset = 57.0; 
      public static final double StowOffset = 0.0;
      public static final double PickupOffset = 0.0;
    }
  }

  public static final class ClimbConstants {

    public static final int MOTOR_1_ID = 20;
    public static final int MOTOR_2_ID = 21;
    public static final int CANCODER_ID = 22;
    public static final double UPPER_SOFT_BOUND = 0.178; 
    public static final double LOWER_SOFT_BOUND = -0.162;
    public static final double GEAR_RATIO = 1.5; 

  }

  public static final class FieldConstants{
    // Left and right are viewed from the DS.
    public static final double RED_X = 10.402;
    public static final double BLUE_X = 7.164;

    public static final Pose2d FAR_RIGHT = new Pose2d(RED_X, 7.145, Rotation2d.fromDegrees(-180));
    public static final Pose2d FAR_LEFT = new Pose2d(RED_X, 1.08, Rotation2d.fromDegrees(-180));
    public static final Pose2d RIGHT = new Pose2d(RED_X, 5.645, Rotation2d.fromDegrees(-180));
    public static final Pose2d LEFT = new Pose2d(RED_X, 2.636, Rotation2d.fromDegrees(-180));
    public static final Pose2d CENTER_RED = new Pose2d(RED_X, 4.0, Rotation2d.fromDegrees(-180));

    public static final Pose2d FAR_LEFT_BLUE = new Pose2d(BLUE_X, 6.92, Rotation2d.fromDegrees(0));
    public static final Pose2d FAR_RIGHT_BLUE = new Pose2d(BLUE_X, 1, Rotation2d.fromDegrees(0));
    public static final Pose2d LEFT_BLUE = new Pose2d(BLUE_X, 5.645, Rotation2d.fromDegrees(0));
    public static final Pose2d RIGHT_BLUE = new Pose2d(BLUE_X, 2.636, Rotation2d.fromDegrees(0));
    public static final Pose2d CENTER_BLUE = new Pose2d(BLUE_X, 4.0, Rotation2d.fromDegrees(0));

    public static Pose2d getFarRight(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? FAR_RIGHT : FAR_RIGHT_BLUE;
    }

    public static Pose2d getFarLeft(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? FAR_LEFT : FAR_LEFT_BLUE;
    }

    public static Pose2d getRight(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? RIGHT : RIGHT_BLUE;
    }

    public static Pose2d getLeft(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? LEFT : LEFT_BLUE;
    }

    public static Pose2d getCenter(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? CENTER_RED : CENTER_BLUE;
    }
  }
}