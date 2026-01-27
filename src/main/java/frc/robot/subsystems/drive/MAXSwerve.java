// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

//imports wpi libraries
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DriveConstants;

public class MAXSwerve {
  // Create MAXSwerveModules
  private final SwerveModuleIO m_frontLeft = createModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModuleIO m_frontRight = createModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModuleIO m_rearLeft = createModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModuleIO m_rearRight = createModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI); 

  private final Odometry m_odometry = new Odometry(m_gyro, this::getModulePositions);
  private Pose2d m_simPose = new Pose2d();
  private double m_simLastTimestamp = Timer.getFPGATimestamp();

  /** Creates a new MAXSwerve. */
  public MAXSwerve() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  public void periodic() {
    m_odometry.update();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPose();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.reset(pose);
    if (!RobotBase.isReal()) {
      m_simPose = pose;
      m_simLastTimestamp = Timer.getFPGATimestamp();
    }
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = ySpeed;
    double ySpeedDelivered = xSpeed;
    double rotDelivered = rot;
    if (!RobotBase.isReal()) {
      m_odometry.updateSimHeading(rotDelivered);
      updateSimPose(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_odometry.zeroHeading();
  }

  public void offset180() {
    m_odometry.offset180();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_odometry.getHeading();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_odometry.getTurnRate();
  }

  private SwerveModuleIO createModule(int driveCanId, int turnCanId, double chassisAngularOffset) {
    if (RobotBase.isReal()) {
      return new MAXSwerveModule(driveCanId, turnCanId, chassisAngularOffset);
    }
    return new SimSwerveModule(chassisAngularOffset);
  }

  private void updateSimPose(double xSpeed, double ySpeed, double omega, boolean fieldRelative) {
    double now = Timer.getFPGATimestamp();
    double dt = now - m_simLastTimestamp;
    m_simLastTimestamp = now;
    ChassisSpeeds speeds =
        fieldRelative
            ? new ChassisSpeeds(xSpeed, ySpeed, omega)
            : ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, m_simPose.getRotation());
    double dxField = speeds.vxMetersPerSecond * dt;
    double dyField = speeds.vyMetersPerSecond * dt;
    double dTheta = speeds.omegaRadiansPerSecond * dt;
    m_simPose = new Pose2d(
        m_simPose.getX() + dxField,
        m_simPose.getY() + dyField,
        m_simPose.getRotation().plus(Rotation2d.fromRadians(dTheta)));
    m_odometry.reset(m_simPose);
  }
}
