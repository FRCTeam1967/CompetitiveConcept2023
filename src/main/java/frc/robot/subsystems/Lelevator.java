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

public class Lelevator extends SubsystemBase {
  private WPI_TalonFX leftMotor;
  private WPI_TalonFX rightMotor;
  /** Creates a new LateralElevator. */
  public Lelevator() {
    leftMotor = new WPI_TalonFX(Constants.LateralElevator.LEFT_MOTOR_IDX);
    rightMotor = new WPI_TalonFX(Constants.LateralElevator.RIGHT_MOTOR_IDX);

    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.configMotionCruiseVelocity(Constants.LateralElevator.CRUISE_VELOCITY);
    leftMotor.configMotionAcceleration(Constants.LateralElevator.ACCELERATION);

    leftMotor.config_kP(Constants.LateralElevator.SLOT_IDX_VALUE, Constants.Elevator.kP);
    leftMotor.config_kI(Constants.LateralElevator.SLOT_IDX_VALUE, Constants.Elevator.kI);
    leftMotor.config_kD(Constants.LateralElevator.SLOT_IDX_VALUE, Constants.Elevator.kD);

    rightMotor.follow(leftMotor);
  }

  public void stopMotors(){
    leftMotor.set(ControlMode.PercentOutput, 0);
    rightMotor.set(ControlMode.PercentOutput, 0);
  }

  public void moveTo(double inches){
    double encoderTicks = inches / Constants.LateralElevator.GEAR_RATIO * Constants.LateralElevator.SPROCKET_PITCH_CIRCUMFERENCE / Constants.LateralElevator.FALCON_ENCODER_TICKS_PER_REVOLUTION;
    leftMotor.set(ControlMode.MotionMagic, 1/encoderTicks, DemandType.ArbitraryFeedForward, Constants.LateralElevator.FEED_FORWARD);
  }

  public boolean atDistance(){
    double error = Math.abs(leftMotor.getClosedLoopError());
    return error < Constants.LateralElevator.ERROR_THRESHOLD; //error threshold
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
