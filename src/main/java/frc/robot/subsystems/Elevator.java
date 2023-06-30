package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    
    private CANSparkMax leftVerticalMotor;
    private CANSparkMax rightVerticalMotor;
    private TrapezoidProfile.Constraints motionProfile = new TrapezoidProfile.Constraints(10,20);
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public Elevator() {
        leftVerticalMotor = new CANSparkMax(Constants.Elevator.LEFT_MOTOR_IDX, MotorType.kBrushless);
        SparkMaxPIDController pidController = leftVerticalMotor.getPIDController();
        pidController.setP(Constants.Elevator.kP);
        pidController.setI(Constants.Elevator.kI);
        pidController.setD(Constants.Elevator.kD);
        pidController.setOutputRange(-0.2, 0.2);
        leftVerticalMotor.setSmartCurrentLimit(40);

        rightVerticalMotor = new CANSparkMax(Constants.Elevator.RIGHT_MOTOR_IDX, MotorType.kBrushless);
        leftVerticalMotor.setSmartCurrentLimit(40);
        rightVerticalMotor.follow(leftVerticalMotor);
    }

    public void stop() {
        leftVerticalMotor.stopMotor();
        rightVerticalMotor.stopMotor();
    }

    public void moveTo(double inches) {
        goal = new TrapezoidProfile.State(inches, 0);
    }

    public boolean isAtHeight(double inches) {
        return true;
    }

    @Override
    public void periodic() {
        var profile = new TrapezoidProfile(motionProfile, goal, setpoint);
        setpoint = profile.calculate(Constants.ROBOT_PERIOD);

        leftVerticalMotor.getPIDController().setReference(setpoint.position, CANSparkMax.ControlType.kPosition);
    }
}
