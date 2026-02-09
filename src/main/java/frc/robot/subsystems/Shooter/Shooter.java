package frc.robot.subsystems.Shooter;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private SparkFlex rightShooter = new SparkFlex(ShooterConstants.right_shooter_id, MotorType.kBrushless);
    private SparkFlex leftShooter = new SparkFlex(ShooterConstants.left_shooter_id, MotorType.kBrushless);

    private double targetRPM = 0;

    @SuppressWarnings({ "removal", "deprecation" })
    public Shooter() {

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

    @Override
    public void periodic() {
    }
}
