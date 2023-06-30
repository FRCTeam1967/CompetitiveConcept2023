package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;


public class SwerveModule {

    //creating both motors
    private CANSparkMax powerController;
    private CANSparkMax steerController;

    public SwerveModule(String name, int powerIdx, int steerIdx) {
        powerController = new CANSparkMax(powerIdx, MotorType.kBrushless);
        //defining PID for power motor
        SparkMaxPIDController powerPIDController = powerController.getPIDController();
        powerPIDController.setP(Constants.Swerve.POWER_kP);
        powerPIDController.setI(Constants.Swerve.POWER_kI);
        powerPIDController.setD(Constants.Swerve.POWER_kD);
        //setting lower power bound at deadband and upper bound at max output
        powerPIDController.setOutputRange(Constants.Swerve.DEFAULT_NEUTRAL_DEADBAND, Constants.Swerve.MAX_OUTPUT);
        //using neo's encoder for power motor feedback
        powerPIDController.setFeedbackDevice(powerController.getEncoder());
        
        steerController = new CANSparkMax(steerIdx, MotorType.kBrushless);
        //defining PID for steer motor
        SparkMaxPIDController steerPIDController = steerController.getPIDController();
        steerPIDController.setP(Constants.Swerve.STEER_kP);
        steerPIDController.setI(Constants.Swerve.STEER_kI);
        steerPIDController.setD(Constants.Swerve.STEER_kD);
        //using canandcoder for steer controller encoder
        var analogEncoder = steerController.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        //set bit size depth for encoder at 16 (usually 16, 32, 64, etc)
        analogEncoder.setAverageDepth(Constants.Swerve.ANALOG_SAMPLE_DEPTH);
        //multiply native encoder units by 360 for each full rev
        analogEncoder.setPositionConversionFactor(360);

        //setting up PID wrapping for the pid controller (min and max at 0 and 360)
        steerPIDController.setPositionPIDWrappingEnabled(true);
        steerPIDController.setPositionPIDWrappingMinInput(0);
        steerPIDController.setPositionPIDWrappingMaxInput(360);

        steerPIDController.setOutputRange(Constants.Swerve.DEFAULT_NEUTRAL_DEADBAND, Constants.Swerve.MAX_OUTPUT);
        steerPIDController.setFeedbackDevice(analogEncoder);
    }

    //not used right now - get current state for module (velocity * gear ratio) and degrees from steer controller
    public SwerveModuleState getState() {
        return new SwerveModuleState(powerController.getEncoder().getVelocity()*Constants.Swerve.OUTPUT_GEAR_RATIO,
        Rotation2d.fromDegrees(steerController.getEncoder().getPosition())); 
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
        powerController.getPIDController().setReference(state.speedMetersPerSecond / Constants.Swerve.OUTPUT_GEAR_RATIO, ControlType.kVelocity);
        steerController.getPIDController().setReference(state.angle.getDegrees(), ControlType.kPosition);
    }

    public void stop() {
        //stop modules (velocity = 0)
        powerController.getPIDController().setReference(0, ControlType.kVelocity);
    }
    
}

