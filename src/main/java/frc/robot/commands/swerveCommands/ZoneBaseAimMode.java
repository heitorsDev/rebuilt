package frc.robot.commands.swerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ZoneBaseAimMode extends Command {
    SwerveSubsystem swerve;

    public ZoneBaseAimMode(SwerveSubsystem swerve){
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        
        Pose2d swervePose = swerve.getPose();
        Alliance alliance = DriverStation.getAlliance().get();
        switch (alliance){
            case Blue:
                if (swervePose.getX()<4){
                    swerve.aimToGoal();        
                } else {
                    swerve.aimForFeeding();
                }
                break;
            case Red:
            //16.54

                if (swervePose.getX()>16.54-4){
                    swerve.aimToGoal();        
                } else {
                    swerve.aimForFeeding();
                }
                break;
            default:
                break;
            
        }


        swerve.aimToGoal();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        
        return swerve.angularPIDStable(5);
    }
}
