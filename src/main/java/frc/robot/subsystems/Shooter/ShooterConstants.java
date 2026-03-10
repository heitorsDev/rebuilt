package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Shooter.Interpolation.ShootingInterpLUT;

public final class ShooterConstants {

    public static final int right_shooter_id = 16;
    public static final int left_shooter_id  = 15;

    public static final double shooterkP = 0.0005;
    public static final double shooterkI = 0.0;
    public static final double shooterkD = 0.000;
    public static final double shooterkV = 0.00018;

    public static final double[][] RPMtable = {
        {2, 3400},
        {2.5, 3700},
        {3.0,4500}
    };

    public static final ShootingInterpLUT RPMinterpolation =
            new ShootingInterpLUT(RPMtable);
}
