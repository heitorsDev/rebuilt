package frc.robot.commands.indexerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.Indexer.INDEXER_STATES;
import frc.robot.subsystems.Intake.Intake.PIVOT_STATES;
import frc.robot.subsystems.Intake.Intake.ROLLER_STATES;

public class IndexCommand extends Command {
    Indexer indexer;
    public IndexCommand(Indexer indexer){
        this.indexer = indexer;
        addRequirements(indexer);
    }
    @Override
    public void initialize() {
        indexer.setIndexerState(INDEXER_STATES.SERIALIZE);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
