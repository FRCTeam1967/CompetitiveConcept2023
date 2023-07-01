// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */



  private CANSparkMax wristMotor;
  
  private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(10,20);
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  public Wrist() {
    wristMotor = new CANSparkMax(Constants.Wrist.WRIST_MOTOR_IDX, MotorType.kBrushless);

    SparkMaxPIDController pidController = wristMotor.getPIDController();
    pidController.setP(Constants.Wrist.kP);
    pidController.setI(Constants.Wrist.kI);
    pidController.setD(Constants.Wrist.kD);
    pidController.setOutputRange(-0.2, 0.2);
    wristMotor.setSmartCurrentLimit(40);
  }

  public void stop() {
    wristMotor.stopMotor();
  }

  public void moveTo(double angle) {
    goal = new TrapezoidProfile.State(angle, 0);
  }

  @Override
  public void periodic() {
    
    var profile = new TrapezoidProfile(motionProfile, goal, setpoint);
    setpoint = profile.calculate(Constants.ROBOT_PERIOD);

    double degreesToRev = (setpoint.position/360) * Constants.Wrist.GEAR_RATIO;

    wristMotor.getPIDController().setReference(degreesToRev, CANSparkMax.ControlType.kPosition);
  
  }
}
