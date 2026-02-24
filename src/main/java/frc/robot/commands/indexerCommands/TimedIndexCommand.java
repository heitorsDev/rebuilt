package frc.robot.commands.indexerCommands;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.shooterCommands.SetShooterState;
import frc.robot.commands.swerveCommands.AlignToGoal;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.SHOOTER_STATES;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TimedIndexCommand extends SequentialCommandGroup{
    public TimedIndexCommand(Indexer indexer, SwerveSubsystem swerve, double time, Shooter shooter){
        addCommands(    
            new SetShooterState(shooter, SHOOTER_STATES.HUB),
            new AlignToGoal(swerve),
            new IndexCommand(indexer),
            new WaitCommand(time),
            new DeIndexCommand(indexer)
        );
        addRequirements(indexer, swerve, shooter);
    }
}
