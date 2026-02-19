package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.indexerCommands.IndexCommand;
import frc.robot.commands.indexerCommands.DeIndexCommand;
import frc.robot.commands.intakeCommands.DropIntakeCommand;
import frc.robot.commands.intakeCommands.InsideIntakeCommand;
import frc.robot.commands.swerveCommands.AimToGoalMode;
import frc.robot.commands.swerveCommands.UnlockDrivingMode;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController opController = new CommandXboxController(1);
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final Shooter shooter = new Shooter(swerve::getPose, opController::getRightY);




  public RobotContainer() {
    NamedCommands.registerCommand("DropIntakeCommand", new DropIntakeCommand(intake));
    NamedCommands.registerCommand("InsideIntakeCommand", new InsideIntakeCommand(intake));
    NamedCommands.registerCommand("IndexCommand", new IndexCommand(indexer));
    NamedCommands.registerCommand("DeIndexCommand", new DeIndexCommand(indexer));
    
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {

    swerve.setDefaultCommand(
        Commands.run(() -> {
          double xSpeed = -driverController.getLeftY();
          double ySpeed = -driverController.getLeftX();
          double rot = -driverController.getRightX();

          swerve.driveTeleop(xSpeed * 4.5, ySpeed * 4.5, rot * 3.5);
        }, swerve));

    driverController.start().onTrue(Commands.runOnce(swerve::zeroGyro));

    driverController.leftTrigger(0.3).toggleOnTrue(new DropIntakeCommand(intake));
    driverController.leftTrigger(0.3).toggleOnFalse(new InsideIntakeCommand(intake));

    driverController.leftBumper().onTrue(new AimToGoalMode(swerve));
    driverController.leftBumper().onFalse(new UnlockDrivingMode(swerve));
    
    driverController.rightBumper().onTrue(new IndexCommand(indexer));
    driverController.rightBumper().onFalse(new DeIndexCommand(indexer));
  }

  public Command getAutonomousCommand() {
    return swerve.getAutonomousCommand("B.classific.auto.lado(human)");
  }
}
