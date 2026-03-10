package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.indexerCommands.IndexCommand;
import frc.robot.commands.indexerCommands.TimedIndexCommand;
import frc.robot.commands.climberCommands.ClimbSequence;
import frc.robot.commands.climberCommands.SetClimberState;
import frc.robot.commands.indexerCommands.DeIndexCommand;
import frc.robot.commands.intakeCommands.DropIntakeCommand;
import frc.robot.commands.intakeCommands.InsideIntakeCommand;
import frc.robot.commands.intakeCommands.MidIntakeCommand;
import frc.robot.commands.shooterCommands.SetShooterState;
import frc.robot.commands.swerveCommands.AimForFeedMode;
import frc.robot.commands.swerveCommands.AimToGoalMode;
import frc.robot.commands.swerveCommands.UnlockDrivingMode;
import frc.robot.commands.swerveCommands.ZoneBaseAimMode;
import frc.robot.commands.teleOpSequences.ShootingSequence;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.Climber.CLIMBER_STATES;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.SHOOTER_STATES;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Field.Field;
import java.util.Optional;

import com.ctre.phoenix.led.Animation;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController opController = new CommandXboxController(1);
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final Field field = new Field();
  private final SwerveSubsystem swerve = new SwerveSubsystem(field::getHubPose);
  private final Shooter shooter = new Shooter(swerve::getPose, field::getHubPose, swerve::getCurrentFeedingPose);
  private final Climber climber = new Climber(opController::getLeftY);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    HttpCamera frontWebCam = new HttpCamera("front-webcam", "http://10.101.90.11:1181/stream.mjpeg");
    CameraServer.addCamera(frontWebCam);

    CameraServer.addServer("limelight-stream")
        .setSource(new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpeg"));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);

    NamedCommands.registerCommand("DropIntakeCommand", new DropIntakeCommand(intake));
    NamedCommands.registerCommand("InsideIntakeCommand", new InsideIntakeCommand(intake));
    NamedCommands.registerCommand("IndexCommand", new IndexCommand(indexer));
    NamedCommands.registerCommand("DeIndexCommand", new DeIndexCommand(indexer));
    NamedCommands.registerCommand("TimedIndexCommand", new TimedIndexCommand(indexer, swerve, shooter, intake, 3));
    // MUDAR DEPOIS
    NamedCommands.registerCommand("ClimbCommand", Commands.runOnce(() -> {
      climber.setClimberState(CLIMBER_STATES.DOWN);
    }, climber));
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

    opController.povUp().onTrue(new SetClimberState(climber, CLIMBER_STATES.UP));
    opController.povDown().onTrue(new ClimbSequence(climber, intake));
    opController.povRight().onTrue(new SetClimberState(climber, CLIMBER_STATES.DOWNDOWN));

    opController.rightTrigger(0.3).onTrue(new DropIntakeCommand(intake));
    opController.leftTrigger(0.3).onFalse(new InsideIntakeCommand(intake));

    driverController.rightTrigger(0.5).onTrue(new MidIntakeCommand(intake));
    driverController.rightTrigger(0.5).onTrue(new DropIntakeCommand(intake)); // perguntar pro enzo

    driverController.rightBumper().onTrue(new ShootingSequence.Hub(indexer, intake, shooter, swerve));
    driverController.rightBumper().onFalse(new ShootingSequence.UnlockAim(indexer, intake, shooter, swerve));

    driverController.leftBumper().onTrue(new ShootingSequence.Feed(indexer, intake, shooter, swerve));
    driverController.leftBumper().onFalse(new ShootingSequence.UnlockAim(indexer, intake, shooter, swerve));

    driverController.leftTrigger(0.5).onTrue(new ShootingSequence.Hub(indexer, intake, shooter, swerve));
    driverController.leftTrigger(0.5).onFalse(new ShootingSequence.UnlockAim(indexer, intake, shooter, swerve));

  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("Human player side (HP)");
  }
}
