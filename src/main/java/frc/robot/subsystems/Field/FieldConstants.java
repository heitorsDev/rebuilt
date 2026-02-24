package frc.robot.subsystems.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d redHubPose  = new Pose2d(14.5, 4, Rotation2d.fromDegrees(180));
    public static final Pose2d blueHubPose = new Pose2d(4.6, 4, Rotation2d.fromDegrees(180));

    public static final Pose2d feedingPoseBlueHP = new Pose2d(2,2, new Rotation2d());
    public static final Pose2d feedingPoseRedHP = new Pose2d(16.54-2, 2, new Rotation2d());
    public static final Pose2d feedingPoseBlueDP = new Pose2d(2, 6, new Rotation2d());
    public static final Pose2d feedingPoseRedDP = new Pose2d(16.54-2, 6, new Rotation2d());
       
}
