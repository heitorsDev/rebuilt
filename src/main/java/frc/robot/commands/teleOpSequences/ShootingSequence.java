package frc.robot.commands.teleOpSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexerCommands.DeIndexCommand;
import frc.robot.commands.indexerCommands.IndexCommand;
import frc.robot.commands.shooterCommands.SetShooterState;
import frc.robot.commands.swerveCommands.AimToGoalMode;
import frc.robot.commands.swerveCommands.UnlockDrivingMode;
import frc.robot.commands.swerveCommands.ZoneBaseAimMode;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.SHOOTER_STATES;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShootingSequence {
    /*
     * driverController.rightBumper().onTrue(new SequentialCommandGroup(
     * new SetShooterState(shooter, SHOOTER_STATES.HUB),
     * new AimToGoalMode(swerve),
     * new IndexCommand(indexer)));
     */
    public static class Feed extends SequentialCommandGroup {
        public Feed(Indexer indexer, Intake intake, Shooter shooter, SwerveSubsystem swerve) {
            addCommands(
                    new SetShooterState(shooter, SHOOTER_STATES.FEED),
                    new AimToGoalMode(swerve),
                    new IndexCommand(indexer));
            addRequirements(indexer, intake,  shooter);
        }
    }

    public static class Hub extends SequentialCommandGroup {
        public Hub(Indexer indexer, Intake intake, Shooter shooter, SwerveSubsystem swerve) {
            addCommands(
                    new SetShooterState(shooter, SHOOTER_STATES.HUB),
                    new AimToGoalMode(swerve),
                    new IndexCommand(indexer));
            addRequirements(indexer, intake, shooter);
        }
    }

    public static class Zoned extends SequentialCommandGroup {
        public Zoned(Indexer indexer, Intake intake, Shooter shooter, SwerveSubsystem swerve) {
            addCommands(
                    new SetShooterState(shooter, SHOOTER_STATES.HUB),
                    new ZoneBaseAimMode(swerve),
                    new IndexCommand(indexer));
            addRequirements(indexer, intake,  shooter);
        }
    }

    public static class UnlockAim extends SequentialCommandGroup {
        public UnlockAim(Indexer indexer, Intake intake, Shooter shooter, SwerveSubsystem swerve) {
            addCommands(
                    new UnlockDrivingMode(swerve),
                    new DeIndexCommand(indexer));
            addRequirements(indexer, intake,  shooter);
        }
    }
}
