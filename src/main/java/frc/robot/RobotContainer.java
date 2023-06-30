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
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Swerve swerve = new Swerve();
  //private final Elevator elevator = new Elevator();
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);

  //will move elevator to 40 in position
  // private final Command elevatorHigh = new RunCommand(() -> {
  //     elevator.moveTo(40);
  // }, elevator);

  // //will move to 50 in and wait until next command 
  // private final Command elevateAndWait = new FunctionalCommand(
  //   () -> {}, 
  //   () -> {elevator.moveTo(50);}, 
  //   x -> {}, 
  //   () -> {return elevator.isAtHeight(50);}, 
  //   elevator
  // );


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //default commands are automatically scheduled - usually isFinished always returns falses
    swerve.setDefaultCommand(new SwerveDrive(swerve, () -> leftJoystick.getRawAxis(0), () -> leftJoystick.getRawAxis(1), () -> rightJoystick.getRawAxis(0)));
    
    //m_driverController.button(4).onTrue(elevatorHigh);
    //move to 50 in and then move to 40 in - sequential commands
    //m_driverController.button(5).onTrue(elevateAndWait.andThen(elevatorHigh));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
