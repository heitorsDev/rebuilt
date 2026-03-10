package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.Climber.CLIMBER_STATES;

public class SetClimberState extends Command{
    Climber climber;
    CLIMBER_STATES state;

    public SetClimberState(Climber climber, CLIMBER_STATES state){
        this.climber = climber;
        this.state = state;
        addRequirements(climber);
    }
    
    @Override
    public void initialize() {
        climber.setClimberState(state);
        
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
