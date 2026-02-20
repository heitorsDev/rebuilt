package frc.robot.commands.swerveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlignToGoal extends Command {

    private static final double angleTreshold = Math.toRadians(2.0);
    private static final double stableTime = 0.2;

    private final SwerveSubsystem swerve;
    private Pose2d targetPose;
    
        private final PIDController angularPID;
        private final Timer stableTimer;
    
        public AlignToGoal(SwerveSubsystem swerve) {
            this.swerve = swerve;
            
            this.stableTimer = new Timer();
    
            this.angularPID = new PIDController(4.0, 0.0, 0.15);
            this.angularPID.enableContinuousInput(-Math.PI, Math.PI);
            this.angularPID.setTolerance(angleTreshold);
    
            addRequirements(swerve);
        }
    
        @Override
        public void initialize() {
            this.targetPose = (DriverStation.getAlliance().get()==Alliance.Red?ShooterConstants.redHubPose:ShooterConstants.blueHubPose);
        angularPID.reset();
        stableTimer.reset();
        stableTimer.stop();
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        double angleToTarget = Math.atan2(
                targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX());

        double currentHeading = currentPose.getRotation().getRadians();
        double error = MathUtil.angleModulus(angleToTarget - currentHeading);

        double rotationOutput = angularPID.calculate(currentHeading, angleToTarget);

        swerve.drive(new ChassisSpeeds(0, 0, rotationOutput));

        if (Math.abs(error) < angleTreshold) {
            if (!stableTimer.isRunning()) {
                stableTimer.start();
            }
        } else {
            stableTimer.reset();
            stableTimer.stop();
        }

    }

    @Override
    public boolean isFinished() {
        return stableTimer.hasElapsed(stableTime);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0));
        stableTimer.stop();

    }
}