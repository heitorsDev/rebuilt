package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public enum ROLLER_STATES {
        ON, OFF
    }
    private ROLLER_STATES currentRollerState = ROLLER_STATES.OFF;

    public enum PIVOT_STATES {
        DEFAULT, DROP
    }
    private PIVOT_STATES currentPivotState = PIVOT_STATES.DEFAULT;

  
    private SparkMax pivot = new SparkMax(IntakeConstants.pivot_id, MotorType.kBrushless);
    private TalonFX roller = new TalonFX(IntakeConstants.roller_id);

    public Intake() {

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.smartCurrentLimit(40);

        ClosedLoopConfig pid = config.closedLoop;

        pid.pid(
            IntakeConstants.kP,
            IntakeConstants.kI,
            IntakeConstants.kD
        );

        pid.outputRange(-1, 1);

        config.encoder.positionConversionFactor(1.0);
        config.encoder.velocityConversionFactor(1.0);

        pivot.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override
    public void periodic() {
        switch (currentRollerState) {
            case ON:
                roller.set(IntakeConstants.intakePower);
                break;
            case OFF:
                roller.set(0);
                break;
        }
        switch (currentPivotState) {
            case DEFAULT:
                pivotSP = IntakeConstants.defaultPivotSP;
                break;

            case DROP:
                pivotSP = IntakeConstants.dropPivotSP;
                break;
        }

        pivot.getClosedLoopController().setSetpoint(
            pivotSP,
            SparkMax.ControlType.kPosition
        );


    }
    double pivotSP = 0;
    public boolean atPosition(){
        return pivot.getClosedLoopController().isAtSetpoint();
    }


    public void setRollerState(ROLLER_STATES state) {
        currentRollerState = state;
    }

    public void setPivotState(PIVOT_STATES state) {
        currentPivotState = state;
    }
}
