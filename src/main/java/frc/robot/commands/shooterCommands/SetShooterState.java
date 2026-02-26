package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.SHOOTER_STATES;

public class SetShooterState extends Command{
    Shooter shooter;
    SHOOTER_STATES state;
    public SetShooterState(Shooter shooter, SHOOTER_STATES state){
        this.shooter = shooter;
        this.state = state;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        shooter.setState(state);
        
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
