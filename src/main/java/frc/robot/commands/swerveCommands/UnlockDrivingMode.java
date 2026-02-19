package frc.robot.commands.swerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class UnlockDrivingMode extends Command {
    SwerveSubsystem swerve;

    public UnlockDrivingMode(SwerveSubsystem swerve){
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.unlockAim();
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
