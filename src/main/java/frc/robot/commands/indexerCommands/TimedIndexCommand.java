package frc.robot.commands.indexerCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexer.Indexer;

public class TimedIndexCommand extends SequentialCommandGroup{
    public TimedIndexCommand(Indexer indexer, double time){

        addCommands(
            new IndexCommand(indexer),
            new WaitCommand(time),
            new DeIndexCommand(indexer)
        );
        addRequirements(indexer);
    }
}
