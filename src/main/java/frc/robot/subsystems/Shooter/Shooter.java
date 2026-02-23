package frc.robot.subsystems.Shooter;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;

    private final SparkFlex rightShooter =
            new SparkFlex(ShooterConstants.right_shooter_id, MotorType.kBrushless);
    private final SparkFlex leftShooter =
            new SparkFlex(ShooterConstants.left_shooter_id, MotorType.kBrushless);

    private double targetRPM = 0;
    private double distance = 0;

    public enum SHOOTER_STATES {
        DEFAULT,
        TUNING
    }

    private SHOOTER_STATES currentShooterState = SHOOTER_STATES.TUNING;

    private final NetworkTable shooterTable =
            NetworkTableInstance.getDefault().getTable("Shooter");

    private final DoubleEntry ntTuningRPM =
            shooterTable.getDoubleTopic("TuningRPM").getEntry(6000);

    private final DoubleEntry ntRealRPMRight =
            shooterTable.getDoubleTopic("RealRPMRight").getEntry(0);

    private final DoubleEntry ntRealRPMLeft = shooterTable.getDoubleTopic("RealRPMLeft").getEntry(0);
    private final DoubleEntry ntTargetRPM =
            shooterTable.getDoubleTopic("TargetRPM").getEntry(0);
    private final DoubleEntry ntDistance =
            shooterTable.getDoubleTopic("Distance").getEntry(0);
    private final DoubleEntry ntRPMError =
            shooterTable.getDoubleTopic("RPMError").getEntry(0);
        Supplier<Pose2d> hubPoseSupplier;
    public Shooter(Supplier<Pose2d> poseSupplier, Supplier<Pose2d> hubPoseSupplier) {
        this.poseSupplier = poseSupplier;
        this.hubPoseSupplier = poseSupplier;

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
    public double getVelocityLeft(){
        return leftShooter.getEncoder().getVelocity();
    }

    public boolean atSpeed(double toleranceRPM) {
        return Math.abs(getVelocityRight() - targetRPM) < toleranceRPM;
    }

    public void setState(SHOOTER_STATES state) {
        this.currentShooterState = state;
    }
    
    private void updateDistance() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        Pose2d botPose = poseSupplier.get();
        Pose2d hubPose = hubPoseSupplier.get();

        double dx = hubPose.getX() - botPose.getX();
        double dy = hubPose.getY() - botPose.getY();
        distance = Math.hypot(dx, dy);
    }

    private void updatePower() {
        switch (currentShooterState) {
            case TUNING -> {
                double tuningRPM = ntTuningRPM.get();
                setVelocity(tuningRPM);
            }
            case DEFAULT -> setVelocity(ShooterConstants.RPMinterpolation.get(distance));
        }
    }

    private void updateTelemetry() {

        ntRealRPMRight.set(getVelocityRight());
        ntRealRPMLeft.set(getVelocityLeft());
        ntTargetRPM.set(targetRPM);
        ntDistance.set(distance);

        shooterTable.getEntry("State").setString(currentShooterState.name());
        shooterTable.getEntry("AtSpeed").setBoolean(atSpeed(100));
    }

    @Override
    public void periodic() {
        updateDistance();
        updatePower();
        updateTelemetry();
        
    }
}