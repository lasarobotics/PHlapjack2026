package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;

class Odometry {
  private final AHRS gyro;
  private final Supplier<SwerveModulePosition[]> modulePositions;
  private final SwerveDriveOdometry odometry;
  private final boolean isReal = RobotBase.isReal();
  private double simHeadingRadians = 0.0;
  private double simLastTimestamp = Timer.getFPGATimestamp();

  Odometry(AHRS gyro, Supplier<SwerveModulePosition[]> modulePositions) {
    this.gyro = gyro;
    this.modulePositions = modulePositions;
    this.odometry =
        new SwerveDriveOdometry(
            DriveConstants.kDriveKinematics, getGyroRotation(), modulePositions.get());
  }

  void update() {
    odometry.update(getGyroRotation(), modulePositions.get());
  }

  Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  void reset(Pose2d pose) {
    odometry.resetPosition(getGyroRotation(), modulePositions.get(), pose);
  }

  void zeroHeading() {
    gyro.reset();
    gyro.setAngleAdjustment(0);
    simHeadingRadians = 0.0;
    simLastTimestamp = Timer.getFPGATimestamp();
  }

  void offset180() {
    gyro.setAngleAdjustment(180);
    simHeadingRadians = Math.PI;
  }

  double getHeading() {
    return getGyroRotation().getDegrees();
  }

  double getTurnRate() {
    return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  Rotation2d getRotation2d() {
    return getGyroRotation();
  }

  void updateSimHeading(double rotRadPerSec) {
    if (isReal) return;
    double now = Timer.getFPGATimestamp();
    double dt = now - simLastTimestamp;
    simHeadingRadians = MathUtil.angleModulus(simHeadingRadians + rotRadPerSec * dt);
    simLastTimestamp = now;
  }

  private Rotation2d getGyroRotation() {
    if (!isReal) {
      Rotation2d rotation = Rotation2d.fromRadians(simHeadingRadians);
      if (DriveConstants.kGyroReversed) {
        rotation = rotation.unaryMinus();
      }
      return rotation;
    }
    Rotation2d rotation = gyro.getRotation2d();
    if (DriveConstants.kGyroReversed) {
      rotation = rotation.unaryMinus();
    }
    return rotation;
  }
}
