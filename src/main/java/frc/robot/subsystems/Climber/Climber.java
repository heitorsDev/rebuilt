package frc.robot.subsystems.Climber;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    
    public enum CLIMBER_STATES {
        DOWN, UP, DOWNDOWN, TEST
    }

    Supplier<Double> doubleSupplier;
    
    private CLIMBER_STATES currentClimberState = CLIMBER_STATES.TEST;

  
    private SparkMax climber = new SparkMax(ClimberConstants.climber_id, MotorType.kBrushless);

    public Climber(Supplier<Double> doubleSupplier) {
        this.doubleSupplier = doubleSupplier;
        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.smartCurrentLimit(40);

        ClosedLoopConfig pid = config.closedLoop;

        pid.pid(
            ClimberConstants.kP,
            ClimberConstants.kI,
            ClimberConstants.kD
        );

        pid.outputRange(-1, 1);

        config.encoder.positionConversionFactor(1.0);
        config.encoder.velocityConversionFactor(1.0);

        climber.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    double testPos = 0;

    @Override
    public void periodic() {
        
        switch (currentClimberState) {
            case DOWN:
                climberSP = ClimberConstants.down_setpoint;
                break;

            case UP:
                climberSP = ClimberConstants.up_setpoint;
                break;
            case TEST:
                climberSP+=doubleSupplier.get();
                break;
            case DOWNDOWN:
                climberSP = ClimberConstants.down_down_setpoint;
                break;

        }

        climber.getClosedLoopController().setSetpoint(
            climberSP,
            SparkMax.ControlType.kPosition
        );
        SmartDashboard.putNumber("Climber position: ", climber.getEncoder().getPosition());

    }
    double climberSP = 0;
    public boolean atPosition(){
        return climber.getClosedLoopController().isAtSetpoint();
    }



    public void setClimberState(CLIMBER_STATES state) {
        currentClimberState = state;
    }
}
