package com.team6560.frc2025;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

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
      public static final double L2 = 0.583; // used to be 1.0
      public static final double L3 = 6.5;
      public static final double L4 = 19.1; // 17.65

      public static final double S_STOW = 0.4;
      public static final double S_L2 = 1.51; // 2.58
       public static final double S_L3 = 8.15;  // 7.65
      public static final double S_L4 = 20.8; // should be 21.28 but needs very good PID to not kill spring
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

      // constant + offset = pos

      // C + 35.0 = -215.6 + 360

      public static final double STOW = 90.0;
      public static final double PICKUP = 225;  
      public static final double L1 = 200; 
      public static final double L2 = 80.0; // -215.6, -223 dunk
      public static final double L4 = 45.0; // good

      public static final double S_STOW = 208.5; // same as normal pickup position
      public static final double S_L2 = 148.5; // assuming no offset, also L3 33.76 
      public static final double S_L4 = 250; 

      public static final double L1Offset = 0.0;
      public static final double L2Offset = 35.0;
      public static final double L4Offset = 57.0; // 30.0
      public static final double StowOffset = 0.0;
      public static final double PickupOffset = 0.0;
    }
  }

  public static final class ClimbConstants {

    public static final int MOTOR_1_ID = 20;
    public static final int MOTOR_2_ID = 21;
    public static final int CANCODER_ID = 22;
    public static final double UPPER_SOFT_BOUND = 0.178; // was .198
    public static final double LOWER_SOFT_BOUND = -0.162; // abs encoder
    public static final double GEAR_RATIO = 1.5; 

  }
}