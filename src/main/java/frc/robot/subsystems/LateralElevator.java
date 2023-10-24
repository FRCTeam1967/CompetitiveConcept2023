// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class LateralElevator extends SubsystemBase {
  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;

  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(10,20);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  
  /** Creates a new LateralElevator. */
  public LateralElevator() {
    leftMotor = new CANSparkMax(Constants.LateralElevator.LEFT_MOTOR_IDX, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.LateralElevator.RIGHT_MOTOR_IDX, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    SparkMaxPIDController pidController = leftMotor.getPIDController();
    leftMotor.getEncoder().setPositionConversionFactor(42); //inches to revolutions

    pidController.setP(Constants.LateralElevator.kP);
    pidController.setI(Constants.LateralElevator.kI);
    pidController.setD(Constants.LateralElevator.kD);
    pidController.setFeedbackDevice(leftMotor.getEncoder());

    rightMotor.follow(leftMotor);
  }

  public void stopMotors(){
    leftMotor.getPIDController().setReference(0, ControlType.kVelocity);
    rightMotor.getPIDController().setReference(0, ControlType.kVelocity);
  }

  public void moveTo(double inches){
    goal = new TrapezoidProfile.State(inches, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    TrapezoidProfile profile = new TrapezoidProfile(motionProfile, goal, setpoint);
    setpoint = profile.calculate(Constants.ROBOT_PERIOD);

    double encoderTicksPerRevolution = setpoint.position / Constants.LateralElevator.GEAR_RATIO * Constants.LateralElevator.SPROCKET_PITCH_CIRCUMFERENCE / Constants.LateralElevator.CANSPARKMAX_ENCODER_TICKS_PER_REVOLUTION;

    leftMotor.getPIDController().setReference(encoderTicksPerRevolution, CANSparkMax.ControlType.kPosition);
  }
}