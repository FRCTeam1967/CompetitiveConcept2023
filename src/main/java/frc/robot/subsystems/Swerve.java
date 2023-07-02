package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.modules.SwerveModule;

public class Swerve extends SubsystemBase {

    //define each module
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final ADIS16470_IMU gyro;
    private final SwerveDriveOdometry odometry;
    private Pose2d pose;
    
    public Swerve() {
        frontLeft = new SwerveModule("FrontLeft", Constants.Swerve.FL_POWER, Constants.Swerve.FL_STEER);
        frontRight = new SwerveModule("FrontRight", Constants.Swerve.FR_POWER, Constants.Swerve.FR_STEER);
        backLeft = new SwerveModule("BackLeft", Constants.Swerve.BL_POWER, Constants.Swerve.BL_STEER);
        backRight = new SwerveModule("BackRight", Constants.Swerve.BR_POWER, Constants.Swerve.BR_STEER);
        
        gyro = new ADIS16470_IMU();
        odometry = new SwerveDriveOdometry(Constants.Swerve.SWERVE_DRIVE_KINEMATICS, getRotation2d(), 
        new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        });
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //takes in degrees and returns rotation object with desired angle
    public Rotation2d getRotation2d() {
        var degrees = -gyro.getAngle() - 90;
        return Rotation2d.fromDegrees(degrees);
    }
    
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(), 
            backLeft.getState(),
            backRight.getState()
        };
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetOdometry (Pose2d pose) {
        odometry.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        }, pose);
    }

    @Override
    public void periodic() {
      frontLeft.periodic();
      frontRight.periodic();
      backLeft.periodic();
      backRight.periodic();

      pose = odometry.update(getRotation2d(), new SwerveModulePosition[] {
        frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
      });

      SmartDashboard.putNumber("gyro", gyro.getAngle());
    }

}
