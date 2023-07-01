package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{

    private CANSparkMax intakeMotor;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Elevator.LEFT_MOTOR_IDX, MotorType.kBrushless);
        SparkMaxPIDController pidController = intakeMotor.getPIDController();
        pidController.setP(Constants.Intake.kP);
        pidController.setI(Constants.Intake.kI);
        pidController.setD(Constants.Intake.kD);
        pidController.setOutputRange(-0.2, 0.2);
        intakeMotor.setSmartCurrentLimit(40);

    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public void runMotor(double speed){
        intakeMotor.getPIDController().setReference(speed, ControlType.kVelocity);
    }
    

}
