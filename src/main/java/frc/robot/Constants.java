// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final class ArmConstants {
    public static final int MOTOR_PORT = 4;

    // These are fake gains; in actuality these must be determined individually for each robot

    // Constants tunable through preferences
    public static final String ARM_KP_KEY = "ArmKP";  // The P gain for the PID controller that drives this arm.
    public static final double DEFAULT_ARM_KP = 3.0; 
    public static final String ARM_KS_KEY = "ArmKS"; // Static motor gain
    public static final double DEFAULT_KS_VOLTS = 0.5;
    public static final String ARM_KG_KEY = "ArmKG"; // Gravity gain
    public static final double DEFAULT_KG_VOLTS = 1.25;
    public static final String ARM_KV_KEY = "ArmKV"; // Velocity gain
    public static final double DEFAULT_KV_VOLTS_PER_SEC_PER_RAD = 0.8;
    public static final String ARM_KA_KEY = "ArmKA"; // Acceleration gain
    public static final double DEFAULT_KA_VOLTS_PER_SEC_SQUARED_PER_RAD = 0.05;
    public static final String ARM_VMAX_KEY = "ArmVmax";
    public static final double DEFAULT_MAX_VELOCITY_RAD_PER_SEC = Units.degreesToRadians(90);
    public static final String ARM_AMAX_KEY = "ArmAmax";
    public static final double DEFAULT_MAX_ACCELERATION_RAD_PER_SEC = Units.degreesToRadians(360);

    public static final double GEAR_RATIO = 1.0d / 200;
    public static final double ARM_RAD_PER_ENCODER_ROTATION = 2.0 * Math.PI * GEAR_RATIO;
    public static final double RPM_TO_RAD_PER_SEC = ARM_RAD_PER_ENCODER_ROTATION / 60;

    // Arm positions.  Horizontal = 0 radians
    public static final double MIN_ANGLE_RADS = Units.degreesToRadians(-45);
    public static final double MAX_ANGLE_RADS = Units.degreesToRadians(120);
    public static final double ARM_OFFSET_RADS = MIN_ANGLE_RADS; // Assume arm starts at rest at lowest position
    public static final double ARM_GOAL_POSITION = Units.degreesToRadians(45);
    public static final double POS_INCREMENT = Units.degreesToRadians(2); // For small adjustments
  }
  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
  public static final class ArmSimConstants {
    public static final double ARM_REDUCTION = 1/ArmConstants.GEAR_RATIO;
    public static final double ARM_MASS_KG = 8.0;
    public static final double ARM_LENGTH_INCHES = 30;
    public static final double ARM_LENGTH_METERS = Units.inchesToMeters(ARM_LENGTH_INCHES);
    public static final double START_ANGLE_RADS = ArmConstants.MIN_ANGLE_RADS;
    public static final int ENCODER_PRR = 4096; // Only used to simulate noise in position measurement
    public static final double ENCODER_DISTANCE_PER_PULSE = 2.0 * Math.PI / ENCODER_PRR * ArmConstants.GEAR_RATIO;
  }

}
