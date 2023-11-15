// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private final String alliance = "blue";
  //private final String alliance = "red";

  private final Swerve swerve = new Swerve();
  // private final Elevator elevator = new Elevator();
  // private final Intake intake = new Intake();
  // private final Wrist wrist = new Wrist();

  // SWERVE COMMANDS

  // ELEVATOR COMMANDS
  /*
   * private final Command elevatorHigh = new RunCommand(() -> {
   * elevator.moveTo(40);
   * }, elevator);
   * 
   * //will move to 50 in and wait until next command
   * private final Command elevateAndWait = new FunctionalCommand(
   * () -> {},
   * () -> {elevator.moveTo(50);},
   * x -> {},
   * () -> {return elevator.isAtHeight(50);},
   * elevator
   * );
   */

  // private final Command intakeCube = new RunCommand(() -> {
  // intake.runMotor(Constants.Intake.INTAKE_CUBE_SPEED);
  // }, intake);

  // private final Command shootCubeHigh = new RunCommand(() -> {
  // intake.runMotor(Constants.Intake.SHOOT_HIGH_CUBE_SPEED);
  // }, intake);

  // private final Command shootCubeMiddle = new RunCommand(() -> {
  // intake.runMotor(Constants.Intake.SHOOT_MIDDLE_CUBE_SPEED);
  // }, intake);

  // private final Command shootCubeLow = new RunCommand(() -> {
  // intake.runMotor(Constants.Intake.SHOOT_LOW_CUBE_SPEED);
  // }, intake);

  // public final Command wristStartConfig = new RunCommand(() -> {
  // wrist.moveTo(Constants.Wrist.STARTING_ANGLE);
  // }, wrist);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
  
    //need to make the turning (drive motor) inversed
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> -m_driverController.getRawAxis(1),
      () -> m_driverController.getRawAxis(0), () -> -m_driverController.getRawAxis(4)));
    
    // m_driverController.button(4).onTrue(elevatorHigh);
    // move to 50 in and then move to 40 in - sequential commands
    // m_driverController.button(5).onTrue(elevateAndWait.andThen(elevatorHigh));
  }

  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(
        Constants.Auto.kMaxSpeedMetersPerSecond,
        Constants.Auto.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(Constants.Swerve.SWERVE_DRIVE_KINEMATICS);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("New Path", new PathConstraints(1, 3));

    var thetaController = new ProfiledPIDController(
        Constants.Auto.kPThetaController, 0, 0, Constants.Auto.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    var swerveControllerCommand = new SequentialCommandGroup(
        new InstantCommand(() -> {
          swerve.resetOdometry(examplePath.getInitialHolonomicPose());
        }),
        new SwerveControllerCommand(
            examplePath,
            swerve::getPose, // Functional interface to feed supplier
            Constants.Swerve.SWERVE_DRIVE_KINEMATICS,

            // Position controllers
            new PIDController(Constants.Auto.kPXController, 0, 0),
            new PIDController(Constants.Auto.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve));

    // Reset odometry to the starting pose of the trajectory.
    swerve.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> swerve.stopModules());
  }
}
