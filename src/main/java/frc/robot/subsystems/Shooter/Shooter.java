package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private Supplier<Pose2d> poseSupplier;

    private SparkFlex rightShooter = new SparkFlex(ShooterConstants.right_shooter_id, MotorType.kBrushless);
    private SparkFlex leftShooter = new SparkFlex(ShooterConstants.left_shooter_id, MotorType.kBrushless);

    private double targetRPM = 0;

    public enum SHOOTER_STATES{
        DEFAULT,
        TUNING
    }
    private SHOOTER_STATES currentShooterState = SHOOTER_STATES.TUNING;

    @SuppressWarnings({ "removal", "deprecation" })
    public Shooter(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

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

        config.encoder.velocityConversionFactor(1.0);

        rightShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftShooter.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        leftShooter.setInverted(false);; 
    }


    public void setVelocity(double rpm) {
        targetRPM = rpm;

        rightShooter.getClosedLoopController().setSetpoint(
            rpm,
            SparkFlex.ControlType.kVelocity
        );

        leftShooter.getClosedLoopController().setSetpoint(
            rpm,
            SparkFlex.ControlType.kVelocity
        );
    }

    public double getVelocity() {
        return rightShooter.getEncoder().getVelocity();
    }

    public boolean atSpeed(double toleranceRPM) {
        return Math.abs(getVelocity() - targetRPM) < toleranceRPM;
    }
    public double distance = 0;
    private void updateDistance(Alliance alliance){
        Pose2d botPose = poseSupplier.get();
        Pose2d hubPose = new Pose2d();
        switch (alliance) {
            case Red:
                hubPose = ShooterConstants.redHubPose;
                break;
            case Blue:
                hubPose = ShooterConstants.blueHubPose;
                break;
        }
        
        double dx = hubPose.getX()-botPose.getX();
        double dy = hubPose.getY()-botPose.getY();
        distance = Math.sqrt(Math.pow(dx, 2)+Math.pow(dy, 2));


    }
    private void updatePower(double distance){
            
        switch (currentShooterState) {
            case TUNING:
                double tuningRPM = SmartDashboard.getNumber("Tuning RPM", 0);
                setVelocity(tuningRPM);
                break;
            case DEFAULT:
                setVelocity(ShooterConstants.RPMinterpolation.get(distance));
                break;
        }
    }

    @Override
    public void periodic() {
        updateDistance(DriverStation.getAlliance().get());
        updatePower(this.distance);

        SmartDashboard.putNumber("Distance to hub", distance);
        SmartDashboard.putNumber("Flywheel target RPM", ShooterConstants.RPMinterpolation.get(distance));
        
    }
}
