package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.SHOOTER_STATES;

public class SetShooterState extends Command{
    Shooter shooter;
    public SetShooterState(Shooter shooter, SHOOTER_STATES state){
        this.shooter = shooter;
        shooter.setState(state);
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        
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
