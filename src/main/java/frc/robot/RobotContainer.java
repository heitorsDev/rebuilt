// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.indexerCommands.IndexCommand;
import frc.robot.commands.indexerCommands.DeIndexCommand;

import frc.robot.commands.intakeCommands.DropIntakeCommand;
import frc.robot.commands.intakeCommands.InsideIntakeCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.rightBumper().onTrue(new DropIntakeCommand(intake));
    m_driverController.leftBumper().onTrue(new InsideIntakeCommand(intake));
    m_driverController.a().onTrue(new IndexCommand(indexer));
    m_driverController.a().onFalse(new DeIndexCommand(indexer));

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
