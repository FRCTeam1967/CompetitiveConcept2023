// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.Joystick;

import frc.robot.commands.SwerveDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class RobotContainer {
  private final Swerve swerve = new Swerve();
  // private final Elevator elevator = new Elevator();
  // private final Intake intake = new Intake();
  // private final Wrist wrist = new Wrist();

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
  //   intake.runMotor(Constants.Intake.INTAKE_CUBE_SPEED);
  // }, intake);


  // private final Command shootCubeHigh = new RunCommand(() -> {
  //   intake.runMotor(Constants.Intake.SHOOT_HIGH_CUBE_SPEED);
  // }, intake);

  // private final Command shootCubeMiddle = new RunCommand(() -> {
  //   intake.runMotor(Constants.Intake.SHOOT_MIDDLE_CUBE_SPEED);
  // }, intake);

  // private final Command shootCubeLow = new RunCommand(() -> {
  //   intake.runMotor(Constants.Intake.SHOOT_LOW_CUBE_SPEED);
  // }, intake);

  // public final Command wristStartConfig = new RunCommand(() -> {
  //   wrist.moveTo(Constants.Wrist.STARTING_ANGLE);
  // }, wrist);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

 
  private void configureBindings() {
    // default commands are automatically scheduled - usually isFinished always
    // returns falses
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> m_driverController.getRawAxis(0),
        () -> m_driverController.getRawAxis(1), () -> m_driverController.getRawAxis(4)));

    // m_driverController.button(4).onTrue(elevatorHigh);
    // move to 50 in and then move to 40 in - sequential commands
    // m_driverController.button(5).onTrue(elevateAndWait.andThen(elevatorHigh));
  }

 
  public Command getAutonomousCommand() {
    return null;
  }
}
