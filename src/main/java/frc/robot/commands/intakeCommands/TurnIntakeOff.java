package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PIVOT_STATES;
import frc.robot.subsystems.Intake.Intake.ROLLER_STATES;

public class TurnIntakeOff  extends Command {
    Intake intake;

    public TurnIntakeOff(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setRollerState(ROLLER_STATES.OFF);
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
