// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Elevator extends SubsystemBase {
  private WPI_TalonFX leftMotor;
  private WPI_TalonFX rightMotor;
  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new WPI_TalonFX(Constants.Elevator.LEFT_MOTOR_IDX);
    rightMotor = new WPI_TalonFX(Constants.Elevator.RIGHT_MOTOR_IDX);

    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.configMotionCruiseVelocity(Constants.Elevator.CRUISE_VELOCITY);
    leftMotor.configMotionAcceleration(Constants.Elevator.ACCELERATION);

    leftMotor.config_kP(Constants.Elevator.SLOT_IDX_VALUE, Constants.Elevator.kP);
    leftMotor.config_kI(Constants.Elevator.SLOT_IDX_VALUE, Constants.Elevator.kI);
    leftMotor.config_kD(Constants.Elevator.SLOT_IDX_VALUE, Constants.Elevator.kD);

    rightMotor.follow(leftMotor);
  }

  public void stopMotors(){
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  public void moveTo(double inches){
    double encoderTicks = inches / Constants.Elevator.GEAR_RATIO * Constants.Elevator.SPROCKET_PITCH_CIRCUMFERENCE / Constants.Elevator.FALCON_ENCODER_TICKS_PER_REVOLUTION;
    leftMotor.set(ControlMode.MotionMagic, 1/encoderTicks, DemandType.ArbitraryFeedForward, Constants.Elevator.FEED_FORWARD);
  }

  public boolean atHeight(){
    double error = Math.abs(leftMotor.getClosedLoopError());
    return error < Constants.Elevator.ERROR_THRESHOLD; //error threshold
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
