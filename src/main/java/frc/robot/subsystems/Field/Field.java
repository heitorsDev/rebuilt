package frc.robot.subsystems.Field;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Field extends SubsystemBase {
    private final NetworkTable fieldTable = NetworkTableInstance.getDefault().getTable("Field");

    private final BooleanPublisher ntActiveHub   = fieldTable.getBooleanTopic("activeHub").publish();
    private final DoublePublisher  ntMatchTime   = fieldTable.getDoubleTopic("matchTime").publish();
    private final DoublePublisher  ntShiftTimer  = fieldTable.getDoubleTopic("shiftTimeRemaining").publish();
    private final StringPublisher  ntShiftLabel  = fieldTable.getStringTopic("currentShift").publish();

    // Shift boundaries (match time counts DOWN from ~150)
    private static final double[] SHIFT_BOUNDARIES = { 130.0, 105.0, 80.0, 55.0, 30.0 };
    private static final String[] SHIFT_NAMES      = {
        "Transition", "Shift 1", "Shift 2", "Shift 3", "Shift 4", "Endgame"
    };

    public Field() {}

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        double matchTime = DriverStation.getMatchTime();
        String gameData  = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default  -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red  -> !redInactiveFirst;
            case Blue ->  redInactiveFirst;
        };

        if      (matchTime > 130) return true;
        else if (matchTime > 105) return shift1Active;
        else if (matchTime > 80)  return !shift1Active;
        else if (matchTime > 55)  return shift1Active;
        else if (matchTime > 30)  return !shift1Active;
        else                      return true;
    }

    /**
     * Returns the index into SHIFT_NAMES for the current match time.
     * 0 = Transition, 1-4 = Shifts, 5 = Endgame
     */
    private int getCurrentShiftIndex(double matchTime) {
        if      (matchTime > 130) return 0; // Transition
        else if (matchTime > 105) return 1; // Shift 1
        else if (matchTime > 80)  return 2; // Shift 2
        else if (matchTime > 55)  return 3; // Shift 3
        else if (matchTime > 30)  return 4; // Shift 4
        else                      return 5; // Endgame
    }

    
    private double getTimeUntilNextShift(double matchTime) {
        for (double boundary : SHIFT_BOUNDARIES) {
            if (matchTime > boundary) {
                return matchTime - boundary;
            }
        }
        return 0.0; 
    }

    @Override
    public void periodic() {
        double matchTime = DriverStation.getMatchTime();
        int shiftIndex   = getCurrentShiftIndex(matchTime);

        ntActiveHub.set(isHubActive());
        ntMatchTime.set(matchTime);
        ntShiftTimer.set(getTimeUntilNextShift(matchTime));
        ntShiftLabel.set(SHIFT_NAMES[shiftIndex]);
    }

    public Pose2d getHubPose() {
        return DriverStation.getAlliance().get() == Alliance.Red
            ? FieldConstants.redHubPose
            : FieldConstants.blueHubPose;
    }
}