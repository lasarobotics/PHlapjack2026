package frc.robot.subsystems.drive;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

interface SwerveModuleIO {
  SwerveModuleState getState();

  SwerveModulePosition getPosition();

  void setDesiredState(SwerveModuleState desiredState);

  void resetEncoders();
}
