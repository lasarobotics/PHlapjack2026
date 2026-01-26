package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

class SimSwerveModule implements SwerveModuleIO {
  private final Rotation2d chassisAngularOffset;
  private SwerveModuleState rawState = new SwerveModuleState(0.0, new Rotation2d());
  private double distanceMeters;
  private double lastTimestamp = Timer.getFPGATimestamp();

  SimSwerveModule(double chassisAngularOffsetRadians) {
    this.chassisAngularOffset = Rotation2d.fromRadians(chassisAngularOffsetRadians);
  }

  @Override
  public SwerveModuleState getState() {
    integrateDistance();
    return new SwerveModuleState(rawState.speedMetersPerSecond, rawState.angle.minus(chassisAngularOffset));
  }

  @Override
  public SwerveModulePosition getPosition() {
    integrateDistance();
    return new SwerveModulePosition(distanceMeters, rawState.angle.minus(chassisAngularOffset));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState corrected = new SwerveModuleState();
    corrected.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    corrected.angle = desiredState.angle.plus(chassisAngularOffset);
    SwerveModuleState optimized = optimizeState(corrected, rawState.angle);
    rawState = optimized;
  }

  @Override
  public void resetEncoders() {
    distanceMeters = 0.0;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  private void integrateDistance() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    distanceMeters += rawState.speedMetersPerSecond * dt;
    lastTimestamp = now;
  }

  private SwerveModuleState optimizeState(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double delta = desiredState.angle.minus(currentAngle).getRadians();
    if (Math.abs(delta) > Math.PI / 2.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.fromRadians(Math.PI)));
    }
    return desiredState;
  }
}
