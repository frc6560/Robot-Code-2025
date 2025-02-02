// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
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

  public static final class ElevatorConstants{
    public static int ELEV_LEFT_ID = 14;
    public static int ELEV_RIGHT_ID = 15;
    public static int ELEV_UPPER_LIMIT_SWITCH_ID = 28;
    public static int ELEV_LOWER_LIMIT_SWITCH_ID = 29;

    public static double ELEV_GEAR_RATIO = 16/108; // random
  }

  public static final class WristConstants{
    public static final int M_ID = 16;
    public static final int SWITCH_ID = 22;  //this is still randomized
    public static final int CANCODER_ID = 17; 
    public static final double GEAR_RATIO = 92.57;
    public static final double UPPER_SOFT_BOUND = 240.0; // placeholder values. these are NOT finalized.
    public static final double LOWER_SOFT_BOUND = -5.0;

    public static final double STOW_ANGLE = 100.0;
    public static final double INTAKE_ANGLE = 0.0;
    public static final double L2_ANGLE = 190.0;
    public static final double L4_ANGLE = 22.0;
  }
}
