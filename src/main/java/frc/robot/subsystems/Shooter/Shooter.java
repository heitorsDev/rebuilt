package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Double> tuningSupplier;
    private final SparkFlex rightShooter =
            new SparkFlex(ShooterConstants.right_shooter_id, MotorType.kBrushless);
    private final SparkFlex leftShooter  =
            new SparkFlex(ShooterConstants.left_shooter_id, MotorType.kBrushless);

    private double targetRPM = 0;
    private double distance = 0;

    public enum SHOOTER_STATES {
        DEFAULT,
        TUNING
    }

    private SHOOTER_STATES currentShooterState = SHOOTER_STATES.TUNING;

    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<Double> tuningSupplier) {
        this.poseSupplier = poseSupplier;
        this.tuningSupplier = tuningSupplier;
        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(60);
        config.idleMode(SparkMaxConfig.IdleMode.kCoast);

        ClosedLoopConfig pid = config.closedLoop;
        pid.pid(
                ShooterConstants.shooterkP,
                ShooterConstants.shooterkI,
                ShooterConstants.shooterkD
        );
        pid.velocityFF(ShooterConstants.shooterkV);
        pid.outputRange(-1, 1);

        rightShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftShooter.setInverted(true);
    }

    public void setVelocity(double rpm) {
        targetRPM = rpm;

        rightShooter.getClosedLoopController().setSetpoint(
                rpm, SparkFlex.ControlType.kVelocity);

        leftShooter.getClosedLoopController().setSetpoint(
                rpm, SparkFlex.ControlType.kVelocity);
    }

    public double getVelocity() {
        return rightShooter.getEncoder().getVelocity();
    }

    public boolean atSpeed(double toleranceRPM) {
        return Math.abs(getVelocity() - targetRPM) < toleranceRPM;
    }

    private void updateDistance() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        Pose2d botPose = poseSupplier.get();
        Pose2d hubPose =
                alliance == DriverStation.Alliance.Red
                        ? ShooterConstants.redHubPose
                        : ShooterConstants.blueHubPose;

        double dx = hubPose.getX() - botPose.getX();
        double dy = hubPose.getY() - botPose.getY();

        distance = Math.hypot(dx, dy);
    }
    public double tuningRPM = 0;
    private void updatePower() {
        switch (currentShooterState) {
            case TUNING -> {
                setVelocity(tuningRPM);
            }

            case DEFAULT -> setVelocity(
                    ShooterConstants.RPMinterpolation.get(distance));
        }
    }

    @Override
    public void periodic() {
        updateDistance();
        updatePower();
        tuningRPM +=tuningSupplier.get();
        SmartDashboard.putNumber("Shooter Distance", distance);
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);
        SmartDashboard.putNumber("Shooter Current RPM", getVelocity());
        
    }
}


