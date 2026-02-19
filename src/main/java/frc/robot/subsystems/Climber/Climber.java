package frc.robot.subsystems.Climber;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    
    public enum CLIMBER_STATES {
        DOWN, UP
    }


    
    private CLIMBER_STATES currentClimberState = CLIMBER_STATES.DOWN;

  
    private SparkMax climber = new SparkMax(ClimberConstants.climber_id, MotorType.kBrushless);

    public Climber() {

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
    @Override
    public void periodic() {
        
        switch (currentClimberState) {
            case DOWN:
                climberSP = ClimberConstants.down_setpoint;
                break;

            case UP:
                climberSP = ClimberConstants.up_setpoint;
                break;
        }

        climber.getClosedLoopController().setSetpoint(
            climberSP,
            SparkMax.ControlType.kPosition
        );

    }
    double climberSP = 0;
    public boolean atPosition(){
        return climber.getClosedLoopController().isAtSetpoint();
    }



    public void setClimberState(CLIMBER_STATES state) {
        currentClimberState = state;
    }
}
