// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    //0.319024 = circumference in meters
    //7.8:1 = flipped drive gear ratio
    public static final double REV_TO_METERS = 0.319024 / 7.8;
    public static final double RPM_TO_MS = REV_TO_METERS / 60;
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

    public static final double GEAR_RATIO = 2/1; //Need to check
    public static final double FEED_FORWARD = 0.07;  //Need to check
    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 1.751 * Math.PI; 

    public static final double ELEVATOR_MAX_HEIGHT = 0.07; 
    public static final int SLOT_IDX_VALUE = 0; 

    public static final double CRUISE_VELOCITY = 0; //Need to check value
    public static final double ACCELERATION = 0; //Need to check value

    public static final int FALCON_ENCODER_TICKS_PER_REVOLUTION = 2048;

    public static final double ERROR_THRESHOLD = 0.5; //need to check

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

  public static class LateralElevator{
    public static final int LEFT_MOTOR_IDX = 11;
    public static final int RIGHT_MOTOR_IDX = 12;

    public static final double kP = 0.55;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double GEAR_RATIO = 2/1; //Need to check
    public static final double FEED_FORWARD = 0.07;  //Need to check
    public static final double SPROCKET_PITCH_CIRCUMFERENCE = 0; //Need to check

    public static final double ELEVATOR_MAX_HEIGHT = 0.07; 
    public static final int SLOT_IDX_VALUE = 0; 

    public static final double CRUISE_VELOCITY = 0; //Need to check value
    public static final double ACCELERATION = 0; //Need to check value

    public static final int FALCON_ENCODER_TICKS_PER_REVOLUTION = 2048;

    public static final double ERROR_THRESHOLD = 0.5; //need to check
  }

  public static class Auto {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }




}
