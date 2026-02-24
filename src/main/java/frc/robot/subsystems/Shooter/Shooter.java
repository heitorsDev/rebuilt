package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Pose2d> feedingPoseSupplier;
    private final Supplier<Pose2d> hubPoseSupplier; // FIX: moved to correct position

    private final SparkFlex rightShooter =
            new SparkFlex(ShooterConstants.right_shooter_id, MotorType.kBrushless);
    private final SparkFlex leftShooter =
            new SparkFlex(ShooterConstants.left_shooter_id, MotorType.kBrushless);

    private double targetRPM = 0;
    private double hubDistance = 0;
    private double feedingDistance = 0;

    public enum SHOOTER_STATES {
        HUB,
        FEED,
        TUNING
    }

    private SHOOTER_STATES currentShooterState = SHOOTER_STATES.TUNING;

    private final NetworkTable shooterTable =
            NetworkTableInstance.getDefault().getTable("Shooter");

    private final DoubleEntry ntTuningRPM =
            shooterTable.getDoubleTopic("TuningRPM").getEntry(6000);

    private final DoubleEntry ntRealRPMRight =
            shooterTable.getDoubleTopic("RealRPMRight").getEntry(0);

    private final DoubleEntry ntRealRPMLeft =
            shooterTable.getDoubleTopic("RealRPMLeft").getEntry(0);

    private final DoubleEntry ntTargetRPM =
            shooterTable.getDoubleTopic("TargetRPM").getEntry(0);

    private final DoubleEntry ntDistance =
            shooterTable.getDoubleTopic("Distance").getEntry(0);

    private final DoubleEntry ntRPMError =
            shooterTable.getDoubleTopic("RPMError").getEntry(0);

    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<Pose2d> hubPoseSupplier, Supplier<Pose2d> feedingPoseSupplier) {
        this.poseSupplier = poseSupplier;
        this.hubPoseSupplier = hubPoseSupplier;       // FIX: was incorrectly assigned poseSupplier
        this.feedingPoseSupplier = feedingPoseSupplier;

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(60);
        config.idleMode(SparkMaxConfig.IdleMode.kCoast);

        ClosedLoopConfig pid = config.closedLoop;
        pid.pid(ShooterConstants.shooterkP, ShooterConstants.shooterkI, ShooterConstants.shooterkD);
        pid.velocityFF(ShooterConstants.shooterkV);
        pid.outputRange(-1, 1);

        rightShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooter.setInverted(true);
        ntTuningRPM.set(5000);
    }

    public void setVelocity(double rpm) {
        targetRPM = rpm;
        rightShooter.getClosedLoopController().setSetpoint(rpm, SparkFlex.ControlType.kVelocity);
        leftShooter.getClosedLoopController().setSetpoint(rpm, SparkFlex.ControlType.kVelocity);
    }

    public double getVelocityRight() {
        return rightShooter.getEncoder().getVelocity();
    }

    public double getVelocityLeft() {
        return leftShooter.getEncoder().getVelocity();
    }

    public boolean atSpeed(double toleranceRPM) {
        return Math.abs(getVelocityRight() - targetRPM) < toleranceRPM;
    }

    public void setState(SHOOTER_STATES state) {
        this.currentShooterState = state;
    }

    private void updateHubDistance() {
        Pose2d botPose = poseSupplier.get();
        Pose2d hubPose = hubPoseSupplier.get();

        double dx = hubPose.getX() - botPose.getX();
        double dy = hubPose.getY() - botPose.getY();
        hubDistance = Math.hypot(dx, dy); // FIX: was already correct, but now hubPoseSupplier is properly assigned
    }

    private void updateFeedingDistance() {
        Pose2d botPose = poseSupplier.get();
        Pose2d feedingPose = feedingPoseSupplier.get();

        double dx = feedingPose.getX() - botPose.getX();
        double dy = feedingPose.getY() - botPose.getY();
        feedingDistance = Math.hypot(dx, dy); // FIX: was writing to hubDistance instead of feedingDistance
    }

    private void updatePower() {
        switch (currentShooterState) {
            case TUNING -> {
                double tuningRPM = ntTuningRPM.get();
                setVelocity(tuningRPM);
            }
            case HUB -> setVelocity(ShooterConstants.RPMinterpolation.get(hubDistance));
            case FEED -> setVelocity(ShooterConstants.RPMinterpolation.get(feedingDistance));
        }
    }

    private void updateTelemetry() {
        ntRealRPMRight.set(getVelocityRight());
        ntRealRPMLeft.set(getVelocityLeft());
        ntTargetRPM.set(targetRPM);
        ntDistance.set(hubDistance);
        ntRPMError.set(getVelocityRight() - targetRPM); // FIX: ntRPMError was never being set

        shooterTable.getEntry("State").setString(currentShooterState.name());
        shooterTable.getEntry("AtSpeed").setBoolean(atSpeed(100));
    }

    @Override
    public void periodic() {
        updateHubDistance();    // FIX: reordered — hub first, then feeding, so hubDistance isn't
        updateFeedingDistance(); //      overwritten before telemetry reads it
        updatePower();
        updateTelemetry();
    }
}