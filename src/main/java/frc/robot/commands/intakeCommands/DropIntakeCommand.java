package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.Intake.PIVOT_STATES;
import frc.robot.subsystems.Intake.Intake.ROLLER_STATES;

public class DropIntakeCommand extends Command {
    Intake intake;

    private boolean isRecovering = false;
    private final Timer recoverTimer = new Timer();
    private static final double RECOVER_DURATION = 0.5; 

    public DropIntakeCommand(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPivotState(PIVOT_STATES.DROP);
        intake.setRollerState(ROLLER_STATES.ON);
        isRecovering = false;
        recoverTimer.stop();
        recoverTimer.reset();
    }
    
    @Override
    public void execute() {
        if (!isRecovering && intake.isStuck()) {
            intake.setPivotState(PIVOT_STATES.INSIDE);
            isRecovering = true;
            recoverTimer.restart();
        }

        if (isRecovering && recoverTimer.hasElapsed(RECOVER_DURATION)) {
            intake.setPivotState(PIVOT_STATES.DROP);
            isRecovering = false;
            recoverTimer.stop();
            recoverTimer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        recoverTimer.stop();
        recoverTimer.reset();
    }

    @Override
    public boolean isFinished() {
        return !isRecovering && intake.atPosition();
    }
}