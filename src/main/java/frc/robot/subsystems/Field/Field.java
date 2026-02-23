package frc.robot.subsystems.Field;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class Field extends SubsystemBase {

    private final NetworkTable fieldTable = NetworkTableInstance.getDefault().getTable("Field");

    private final BooleanEntry ntActiveHub = fieldTable.getBooleanTopic("activeHub", getEntry(false));

    public Field() {

    }

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            return true;
        } else if (matchTime > 105) {
            return shift1Active;
        } else if (matchTime > 80) {
            return !shift1Active;
        } else if (matchTime > 55) {
            return shift1Active;
        } else if (matchTime > 30) {
            return !shift1Active;
        } else {
            return true;
        }
    }

     @Override 
     public void periodic() {
        ntActiveHub.set(isHubActive());
    }
}
