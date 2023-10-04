// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax.ControlType;


public class Intake extends SubsystemBase {
  
  private CANSparkMax motor;
  /** Creates a new Intake. */
  public Intake() {
    motor = new CANSparkMax(Constants.Intake.INTAKE_MOTOR_IDX, MotorType.kBrushless);
    SparkMaxPIDController pidController = motor.getPIDController();
    pidController.setP(Constants.Intake.kP);
    pidController.setI(Constants.Intake.kI);
    pidController.setD(Constants.Intake.kD);
    pidController.setOutputRange(-0.2, 0.2);
    motor.setSmartCurrentLimit(40);
  }

  public void runIntake(double x){
    motor.getPIDController().setReference(x, ControlType.kVelocity);
  }

  public void stop(){
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
