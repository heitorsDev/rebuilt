package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.io.File;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.swerve.LimelightHelpers.PoseEstimate;
import swervelib.*;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;

  // Field2d para visualização no dashboard
  private final Field2d field = new Field2d();

  public enum DRIVING_STATES {
    TELE,
    AUTO,
    AUTO_HEADING
  }

  private DRIVING_STATES drivingState = DRIVING_STATES.TELE;
  private Pose2d poseToAim = new Pose2d(0, 0, new Rotation2d(0));

  PIDController angularPID = new PIDController(3, 0, 0);

  public void unlockAim() {
    this.drivingState = DRIVING_STATES.TELE;
  }

  public void aimToGoal() {
    this.aimToPose(hubPoseSupplier.get());
  }

  private void aimToPose(Pose2d poseToAim) {
    this.poseToAim = poseToAim;
    this.drivingState = DRIVING_STATES.AUTO_HEADING;
  }

  public void driveTeleop(double xSpeed, double ySpeed, double rot) {
    switch (drivingState) {
      case TELE:
        this.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot,
                this.getHeading()));
        break;
      case AUTO:
        break;
      case AUTO_HEADING:
        double angleToTarget = Math.atan2(
            this.poseToAim.getY() - this.getPose().getY(),
            this.poseToAim.getX() - this.getPose().getX());
        double angleToAim = MathUtil.angleModulus(angleToTarget + Math.PI);
        angularPID.enableContinuousInput(-Math.PI, +Math.PI);
        this.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed,
                angularPID.calculate(this.getHeading().getRadians(), angleToAim),
                this.getHeading()));
        break;
    }
  }
  private Supplier<Pose2d> hubPoseSupplier;
  public SwerveSubsystem(Supplier<Pose2d> hubPoseSupplier) {
    this(new File(Filesystem.getDeployDirectory(), "swerve"));
    this.hubPoseSupplier = hubPoseSupplier;
  }

  public SwerveSubsystem(File directory) {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(Constants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException("Erro ao inicializar Swerve", e);
    }

    configureSwerve();
    setupPathPlanner();

    // Registra o Field2d no SmartDashboard/Elastic
    SmartDashboard.putData("Field", field);

    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureSwerve() {
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
  }

  @Override
  public void periodic() {
    UpdateVision();
    updateDashboardField();

  }

  /* ======================== VISÃO ======================== */

  private Pose2d lastVisionPose = new Pose2d(0, 0, new Rotation2d(0));

  private void UpdateVision() {
    LimelightHelpers.SetRobotOrientation(
        "limelight",
        swerveDrive.getPose().getRotation().getDegrees(),
        0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = new PoseEstimate();
    Alliance alliance = DriverStation.getAlliance().get();
    switch (alliance) {
      case Red:
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("limelight");
      case Blue:
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }
    if (mt2 == null || mt2.pose == null)
      return;
    if (mt2.tagCount == 0)
      return;

    if (Math.abs(swerveDrive.getGyro().getYawAngularVelocity()
        .in(DegreesPerSecond)) > 720)
      return;

    double avgDist = mt2.avgTagDist;
    double xyStdDev = 0.3 + (avgDist * avgDist * 0.05);
    if (mt2.tagCount > 1)
      xyStdDev *= 0.5;

    swerveDrive.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
    lastVisionPose = mt2.pose;
    swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
  }

  private void updateDashboardField() {
    Pose2d robotPose = getPose();

    field.setRobotPose(robotPose);

    field.getObject("VisionPose").setPose(lastVisionPose);

    if (drivingState == DRIVING_STATES.AUTO_HEADING) {
      field.getObject("AimTarget").setPose(poseToAim);
    } else {
      field.getObject("AimTarget").setPose(new Pose2d(-1, -1, new Rotation2d(0)));
    }
  }

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);

    } catch (Exception e) {
      e.printStackTrace();
    }

    PathfindingCommand.warmupCommand().schedule();
  }

  public void setActiveTrajectory(Trajectory trajectory) {
    field.getObject("Trajectory").setTrajectory(trajectory);
    swerveDrive.postTrajectory(trajectory);
  }

  public Command getAutonomousCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(),
        4.0,
        swerveDrive.getMaximumChassisAngularVelocity(),
        Units.degreesToRadians(720));

    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public Command driveCommand(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega) {
    return run(() -> swerveDrive.drive(
        SwerveMath.scaleTranslation(
            new Translation2d(x.getAsDouble(), y.getAsDouble()),
            swerveDrive.getMaximumChassisVelocity()),
        Math.pow(omega.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
        true, false));
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  /* ======================== ODOMETRIA ======================== */

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
  }

  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /* ======================== UTIL ======================== */

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }
}