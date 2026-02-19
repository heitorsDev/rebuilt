package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Shooter.Interpolation.ShootingInterpLUT;

public final class ShooterConstants {

    public static final int right_shooter_id = 16;
    public static final int left_shooter_id  = 15;

    public static final double shooterkP = 0.0004;
    public static final double shooterkI = 0.0;
    public static final double shooterkD = 0.0001;
    public static final double shooterkV = 0.00018;

    public static final Pose2d redHubPose  = new Pose2d(14.5, 4, Rotation2d.fromDegrees(180));
    public static final Pose2d blueHubPose = new Pose2d(4, 4, Rotation2d.fromDegrees(180));

    public static final double[][] RPMtable = {
        {1.0, 2500},
        {2.0, 3000},
        {3.0, 3500},
        {4.0, 4100},
        {5.0, 4600}
    };

    public static final ShootingInterpLUT RPMinterpolation =
            new ShootingInterpLUT(RPMtable);
}
