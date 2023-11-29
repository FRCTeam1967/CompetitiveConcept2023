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
    private SparkMaxAbsoluteEncoder analogEncoder;

    private String name;

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
        analogEncoder = steerController.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
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
        Rotation2d.fromDegrees(analogEncoder.getPosition())); 
    }

    //this is wrong - analog encoder should not be used to measure translational motion
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            powerController.getEncoder().getPosition()*Constants.Swerve.REV_TO_METERS, getState().angle);
    }

    public static double positiveModulus(double dividend, double divisor) {
        return ((dividend % divisor) + divisor) % divisor;
    }

    // public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    //     double targetAngle = nearestEquivalentAngle(currentAngle.getRadians(), desiredState.angle.getRadians());
    //     double targetSpeed = desiredState.speedMetersPerSecond;
    //     double delta = targetAngle - currentAngle.getRadians();
    //     // We know the targetAngle is within 180 deg of the current angle. If the
    //     // difference is more than 90 deg, there is a closer angle in the opposite
    //     // direction we can use if we reverse the desired speed. For example, suppose
    //     // the wheel is current at 0 deg, and we want to go forward 1m/s with an
    //     // angle of 135 deg. Rather than turning the wheel 135 deg to get to 135
    //     // deg, we could turn the wheel to -45 (45 deg in the other direction) and drive
    //     // backward at 1m/s.
    //     if (Math.abs(delta) > Math.PI / 2) {
    //         targetSpeed = -targetSpeed;
    //         targetAngle = delta > Math.PI / 2 ? (targetAngle - Math.PI) : (targetAngle + Math.PI);
    //     }
        
    //     // //logging
    //     // System.out.println("Current Module Angle: " + currentAngle.getDegrees());
    //     // System.out.println("Desired Module Angle: " + desiredState.angle.getDegrees());
    //     // System.out.println("Optimized Module Angle: " + (nearestEquivalentAngle(currentAngle.getRadians(), desiredState.angle.getRadians())));

    //     return new SwerveModuleState(targetSpeed, Rotation2d.fromRadians(targetAngle));
    // }

    // private static double nearestEquivalentAngle(double scopeReference, double newAngle) {
    //     double lowerOffset = positiveModulus(scopeReference, 2.0 * Math.PI);
    //     // lowerBound is scopeReference/2pi: how many positive wraps around the circle
    //     // aka the 0 degrees in the multiple of revolutions we're in
    //     // e.g., if we were doing math in degrees and the scope reference was 365 deg:
    //     // scopeReference = 365 scopeReference = -450
    //     // lowerOffset = 5 lowerOffset = 270
    //     // lowerBound = 360 lowerBound = -450 - 270 = -720
    //     // upperBound = 720 upperBound = -450 + (360 - 270) = -450 + 90 = -360
    //     // logica band: [360,720] logical band: [-720, -360]
    //     double lowerBound = scopeReference - lowerOffset;
    //     double upperBound = scopeReference + (2.0 * Math.PI - lowerOffset);
    //     // Adjust newAngle so that it's in the range [lowerBound, upperBound]
    //     // IOW, get newAngle into the same [0,360] band as scopeReference
    //     // We take the mod of 2pi to get it in the range [0, 2pi] and then add in the
    //     // lowerBound.
    //     newAngle = positiveModulus(newAngle,  2.0 * Math.PI) + lowerBound;
    //     // Now newAngle is in the same 360 degree band as the reference. However, if
    //     // it's more than 180 degrees apart,
    //     // there's a closer angle in the band above or below the one scopReference is
    //     // in. For example, if the
    //     // reference angle is 5 degrees, and new angle is 359. -1 degrees is actually
    //     // closer.
    //     if (newAngle - scopeReference > Math.PI) {
    //         // If newAngle is more than pi (180 deg) larger than the reference, substract
    //         // 2pi (360 deg),
    //         // as that angle will be less movement
    //         newAngle -= 2.0 * Math.PI;
    //     } else if (newAngle - scopeReference < -Math.PI) {
    //         // If newAngle is less than the reference by more than pi (180 deg), add 2pi
    //         // (360 deg),
    //         // as that angle will be less movement
    //         newAngle += 2.0 * Math.PI;
    //     }
    //     // Return the adjusted angle, which should be within pi (180 deg) of the
    //     // reference.
    //     return newAngle;
    // }


    //old optimize function - didn't work
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double delta = (desiredState.angle.getDegrees() - currentAngle.getDegrees()) % 360;
        if (delta > 180.0) {
            delta += -360;
        } else if (delta < -180.0) {
            delta += 360.0;
        }

        double targetAngle_deg = currentAngle.getDegrees() + delta;

        double targetSpeed_mps = desiredState.speedMetersPerSecond;

        if (delta > 90.0) {
            targetSpeed_mps = -targetSpeed_mps;
            targetAngle_deg += -180.0;
        } else if (delta < -90.0) {
            targetSpeed_mps = -targetSpeed_mps;
            targetAngle_deg += 180.0;
        }

        return new SwerveModuleState(targetSpeed_mps, Rotation2d.fromDegrees(targetAngle_deg));
    }

    //takes in the desired state of module and sets it to the current state
    public void setState(SwerveModuleState state) {
        
        //minimize the change in heading of motor (ex: turn 90 deg instead of 270)
        state.angle = Rotation2d.fromDegrees((state.angle.getDegrees() + 360) % 360);

        // if (this.name == "FrontLeft"){
        //     System.out.println("Desired angle: " + state.angle);
        // }

        SwerveModuleState optimizedState = optimize(state, getState().angle);

        if (this.name == "FrontLeft"){
            System.out.print("Current Angle: " + getState().angle.getDegrees());
            System.out.print("Desired angle: " + state.angle.getDegrees());
            System.out.println(" Optimize angle: " + optimizedState.angle.getDegrees());
        }

        //System.out.println("Optimized angle: " + state.angle);
        //set reference - set desired power and steer reference for each pid controller
        powerController.getPIDController().setReference(optimizedState.speedMetersPerSecond / Constants.Swerve.RPM_TO_MS, ControlType.kVelocity);
        steerController.getPIDController().setReference(optimizedState.angle.getDegrees(), ControlType.kPosition);

    }

    public void stop() {
        //stop modules (velocity = 0)
        powerController.getPIDController().setReference(0, ControlType.kVelocity);
    }
    
    public void periodic() {
        
    }
}

