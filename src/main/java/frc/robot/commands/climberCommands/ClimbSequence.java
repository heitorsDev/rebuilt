package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakeCommands.DropIntakeCommand;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.Climber.CLIMBER_STATES;
import frc.robot.subsystems.Intake.Intake;

public class ClimbSequence extends SequentialCommandGroup{
    public ClimbSequence(Climber climber, Intake intake){
        
        addCommands(
            new DropIntakeCommand(intake),
            new SetClimberState(climber, CLIMBER_STATES.DOWN)
        );
    }
}
