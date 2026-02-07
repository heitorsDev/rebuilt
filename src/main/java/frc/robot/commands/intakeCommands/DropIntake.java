package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PIVOT_STATES;
import frc.robot.subsystems.Intake.Intake.ROLLER_STATES;

public class DropIntake extends Command {
    Intake intake;

    public DropIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotState(PIVOT_STATES.DROP);
        intake.setRollerState(ROLLER_STATES.ON);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return intake.atPosition();
    }
}
