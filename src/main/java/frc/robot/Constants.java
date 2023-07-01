// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final double ROBOT_PERIOD = 0.02;
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Swerve {
    public static final double POWER_kP = 0.0001;
    public static final double POWER_kI = 0;
    public static final double POWER_kD = 0;

    public static final double STEER_kP = 0.03;
    public static final double STEER_kI = 0;
    public static final double STEER_kD = 0;

    public static final double DEFAULT_NEUTRAL_DEADBAND = 0.05;
    public static final double MAX_OUTPUT = 0.7;
    public static final int ANALOG_SAMPLE_DEPTH = 16;

    //0.0053 = circumference in meters/60
    //7.8:1 = flipped drive gear ratio
    public static final double OUTPUT_GEAR_RATIO = 0.0053 / 7.8;
    // public static final double COUNTS_PER_100MS = 4201;

    public static final int GYRO_PORT = 0;

    public static final int FL_POWER = 7;
    public static final int FL_STEER = 5;
    public static final int FR_POWER = 6;
    public static final int FR_STEER = 4;
    public static final int BL_POWER = 11;
    public static final int BL_STEER = 9;
    public static final int BR_POWER = 10;
    public static final int BR_STEER = 8;

    public static final double SWERVE_MAX_SPEED = 3.87096;
    public static final double SWERVE_ROTATION_MAX_SPEED = 10;
    public static final double SWERVE_DEADBAND = 0.05;

    public static final double WIDTH = Units.inchesToMeters(26);
    public static final double LENGTH = Units.inchesToMeters(26);

    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4) * Math.PI;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(-LENGTH / 2, WIDTH / 2),
        new Translation2d(-LENGTH / 2, -WIDTH / 2),
        new Translation2d(LENGTH / 2, WIDTH / 2),
        new Translation2d(LENGTH / 2, -WIDTH / 2));

  }

  public static class Elevator {
    public static final int LEFT_MOTOR_IDX = 8;
    public static final int RIGHT_MOTOR_IDX = 9;

    public static final double kP = 0.55;
    public static final double kI = 0;
    public static final double kD = 0;

  }

  public static class Wrist {
    public static final int WRIST_MOTOR_IDX = 8;

    public static final double kP = 0.55;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double GEAR_RATIO = 0.1;

    public static final double STARTING_ANGLE = 45;

  }

  public static class Intake {
    public static final int INTAKE_MOTOR_IDX = 8;

    public static final double kP = 0.55;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double INTAKE_CUBE_SPEED = -0.2;
    public static final double SHOOT_HIGH_CUBE_SPEED = 0.5;
    public static final double SHOOT_MIDDLE_CUBE_SPEED = 0.3;
    public static final double SHOOT_LOW_CUBE_SPEED = 0.2;

  }


}
