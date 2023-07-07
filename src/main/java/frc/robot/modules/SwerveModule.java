package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;


public class SwerveModule {

    //creating both motors
    private CANSparkMax powerController;
    private CANSparkMax steerController;

    private String name;

    private SparkMaxAbsoluteEncoder analogEncoder = steerController.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    public SwerveModule(String name, int powerIdx, int steerIdx) {
        this.name = name;
        powerController = new CANSparkMax(powerIdx, MotorType.kBrushless);
        powerController.restoreFactoryDefaults();
        //defining PID for power motor
        SparkMaxPIDController powerPIDController = powerController.getPIDController();
        powerPIDController.setP(Constants.Swerve.POWER_kP);
        powerPIDController.setI(Constants.Swerve.POWER_kI);
        powerPIDController.setD(Constants.Swerve.POWER_kD);
        //setting lower power bound at deadband and upper bound at max output
        powerPIDController.setOutputRange(-Constants.Swerve.MAX_OUTPUT, Constants.Swerve.MAX_OUTPUT);
        //using neo's encoder for power motor feedback
        powerPIDController.setFeedbackDevice(powerController.getEncoder());
        
        steerController = new CANSparkMax(steerIdx, MotorType.kBrushless);
        steerController.restoreFactoryDefaults();
        //defining PID for steer motor
        SparkMaxPIDController steerPIDController = steerController.getPIDController();
        steerPIDController.setP(Constants.Swerve.STEER_kP);
        steerPIDController.setI(Constants.Swerve.STEER_kI);
        steerPIDController.setD(Constants.Swerve.STEER_kD);
        //using canandcoder for steer controller encoder
        //var analogEncoder = steerController.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        //set bit size depth for encoder at 16 (usually 16, 32, 64, etc)
        analogEncoder.setAverageDepth(Constants.Swerve.ANALOG_SAMPLE_DEPTH);
        //multiply native encoder units by 360 for each full rev
        analogEncoder.setPositionConversionFactor(360);

        //setting up PID wrapping for the pid controller (min and max at 0 and 360)
        steerPIDController.setPositionPIDWrappingEnabled(true);
        steerPIDController.setPositionPIDWrappingMinInput(0);
        steerPIDController.setPositionPIDWrappingMaxInput(360);

        steerPIDController.setOutputRange(-Constants.Swerve.MAX_OUTPUT, Constants.Swerve.MAX_OUTPUT);
        steerPIDController.setFeedbackDevice(analogEncoder);

        powerController.burnFlash();
        steerController.burnFlash();

        powerController.stopMotor();
        steerController.stopMotor();
    }

    //get current state for module (velocity * gear ratio) and degrees from steer controller
    public SwerveModuleState getState() {
        return new SwerveModuleState(powerController.getEncoder().getVelocity()*Constants.Swerve.RPM_TO_MS,
        Rotation2d.fromDegrees(steerController.getEncoder().getPosition())); 
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            analogEncoder.getPosition()*Constants.Swerve.REV_TO_METERS, getState().angle);
      }

    //takes in the desired state of module and sets it to the current state
    public void setState(SwerveModuleState state) {
        // if (state.speedMetersPerSecond * Constants.Swerve.COUNTS_PER_100MS < 400) {
        //     stop();
        //     return;
        // }

        //minimize the change in heading of motor (ex: turn 90 deg instead of 270)
        state = SwerveModuleState.optimize(state, getState().angle);
        //set reference - set desired power and steer reference for each pid controller
        powerController.getPIDController().setReference(state.speedMetersPerSecond / Constants.Swerve.RPM_TO_MS, ControlType.kVelocity);
        steerController.getPIDController().setReference(state.angle.getDegrees(), ControlType.kPosition);

    }

    public void stop() {
        //stop modules (velocity = 0)
        powerController.getPIDController().setReference(0, ControlType.kVelocity);
    }
    
    public void periodic() {
        
    }
}

