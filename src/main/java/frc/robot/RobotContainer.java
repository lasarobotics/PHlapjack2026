// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController PRIMARY_CONTROLLER =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    zeroRelativeEncoders();
    CameraServer.startAutomaticCapture();
    // CvSink cvsink = CameraServer.getVideo();
    // CvSource outputStream = CameraServer.putVideo("bLUR", 640, 480);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driveSubsystem.configureBindings(
      () -> {
        double leftX = PRIMARY_CONTROLLER.getLeftX();
        Logger.recordOutput("RobotContainer/Inputs/LeftX", leftX);
        return MathUtil.applyDeadband(-leftX, Constants.Swerve.DEADBAND);
      },    // drive left and right
      () -> {
        double leftY = PRIMARY_CONTROLLER.getLeftY();
        Logger.recordOutput("RobotContainer/Inputs/LeftY", leftY);
        return MathUtil.applyDeadband(-leftY, Constants.Swerve.DEADBAND);
      },    // drive forward and back
      () -> {
        double rightX = PRIMARY_CONTROLLER.getRightX();
        Logger.recordOutput("RobotContainer/Inputs/RightX", rightX);
        return MathUtil.applyDeadband(-rightX, Constants.Swerve.DEADBAND);
      },    // drive rotate
      PRIMARY_CONTROLLER.x(),
      PRIMARY_CONTROLLER.b(),
      PRIMARY_CONTROLLER.a()
    );
  }

  private void zeroRelativeEncoders() {
    m_driveSubsystem.zeroOdometry();
  }
}
