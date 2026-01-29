package frc.robot.subsystems.drive;

import java.io.File;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import org.lasarobotics.fsm.StateMachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

    private final MAXSwerve swerveDrive;

    public enum DriveSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        AUTO {
            Timer m_autoTimer = new Timer();

            @Override
            public void initialize() {
                m_autoTimer.reset();
                m_autoTimer.start();
            }

            @Override
            public void execute() {
                if (!m_autoTimer.hasElapsed(Constants.Swerve.AUTO_DRIVE_TIME)) {
                    // left stick backwards 0.25
                    // (towards the driver station)
                    // horizontally centered
                    // no rotation
                    getInstance().drive(0, Constants.Swerve.AUTO_FORWARD_SPEED, 0);
                } else {
                    getInstance().drive(0, 0, 0);
                }
            }

            @Override
            public SystemState nextState() {
                if (DriverStation.isAutonomous()) return this;

                return TELEOP;
            }
        },
        TELEOP {
            @Override
            public void execute() {
                getInstance().drive();
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        }
    }

    private static DriveSubsystem s_driveSubsystemInstance;
    private DoubleSupplier m_leftX;
    private DoubleSupplier m_leftY;
    private DoubleSupplier m_rightX;
    private BooleanSupplier m_reset;
    private BooleanSupplier m_autoSequence;

    private boolean m_autoRunning;
    private boolean m_autoFinishedWhileHeld;
    private int m_autoStage;
    private Pose2d[] m_autoTargets = new Pose2d[3];
    private double m_autoLeftX;
    private double m_autoLeftY;
    private double m_autoRightX;
    private boolean m_autoOverrideActive;
    private static final double[] AUTO_STAGE_MAX_SPEED_MPS = {1.0, 1.0, 1.0};
    private static final double AUTO_STAGE_LINE_OFFSET_METERS = 0.9271;
    private static final double AUTO_STAGE_HOLD_SEC = 0.0;
    private static final double AUTO_STAGE_TIMEOUT_SEC = 2.5;
    private static final double AUTO_POSITION_TOLERANCE = 0.1;
    private static final double AUTO_HEADING_TOLERANCE_RAD = Math.toRadians(8.0);
    private boolean m_autoStageWaiting;
    private double m_autoStageHoldStart;
    private double m_autoStageTimeoutStart;
    private Pose2d m_autoStageStartPose = new Pose2d();

    public static DriveSubsystem getInstance() {
        if (s_driveSubsystemInstance == null) {
            s_driveSubsystemInstance = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        return s_driveSubsystemInstance;
    }

    public DriveSubsystem(
            File directory) {
        super(DriveSubsystemStates.AUTO);
        swerveDrive = new MAXSwerve();
    }

    public void configureBindings(
            DoubleSupplier leftX,
            DoubleSupplier leftY,
            DoubleSupplier rightX,
            BooleanSupplier reset,
            BooleanSupplier autoSequence) {
        m_leftX = leftX;
        m_leftY = leftY;
        m_rightX = rightX;
        m_reset = reset;
        m_autoSequence = autoSequence;
    }

    public void drive() {
        updateAutoSequence();

        double commandedLeftX =
            m_autoOverrideActive
                ? m_autoLeftX
                : m_leftX.getAsDouble()
                    * Constants.Swerve.GIMP_SCALE
                    * Constants.DriveConstants.kMaxSpeedMetersPerSecond
                    * Constants.Swerve.TRANSLATION_SCALE;
        double commandedLeftY =
            m_autoOverrideActive
                ? m_autoLeftY
                : m_leftY.getAsDouble()
                    * Constants.Swerve.GIMP_SCALE
                    * Constants.DriveConstants.kMaxSpeedMetersPerSecond
                    * Constants.Swerve.TRANSLATION_SCALE;
        double commandedRightX =
            m_autoOverrideActive
                ? m_autoRightX
                : m_rightX.getAsDouble()
                    * Constants.Swerve.GIMP_SCALE
                    * Constants.DriveConstants.kMaxAngularSpeed;

        Logger.recordOutput("DriveSubsystem/Inputs/LeftX", commandedLeftX);
        Logger.recordOutput("DriveSubsystem/Inputs/LeftY", commandedLeftY);
        Logger.recordOutput("DriveSubsystem/Inputs/RightX", commandedRightX);

        swerveDrive.drive(
            commandedLeftX,
            commandedLeftY,
            commandedRightX,
            true
        );
    }

    public void drive(double leftX, double leftY, double rightX) {
        Logger.recordOutput("DriveSubsystem/DriveManual/LeftX", m_leftX.getAsDouble());
        Logger.recordOutput("DriveSubsystem/DriveManual/LeftY", m_leftY.getAsDouble());
        Logger.recordOutput("DriveSubsystem/DriveManual/RightX", m_rightX.getAsDouble());
        
        swerveDrive.drive(
            leftX * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            leftY * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            rightX * Constants.Swerve.GIMP_SCALE * Constants.DriveConstants.kMaxAngularSpeed,
            true
        );
    } 

    public void periodic() {
        if (m_reset.getAsBoolean()) {
            zeroOdometry();
        }
        swerveDrive.periodic();
        SwerveModulePosition[] modulePositions = swerveDrive.getModulePositions();
        Logger.recordOutput("DriveSubsystem/Odometry/Pose", swerveDrive.getPose());
        Logger.recordOutput("DriveSubsystem/Odometry/HeadingDegrees", swerveDrive.getHeading());
        Logger.recordOutput(
            "DriveSubsystem/Odometry/WheelDistancesMeters",
            new double[] {
                modulePositions[0].distanceMeters,
                modulePositions[1].distanceMeters,
                modulePositions[2].distanceMeters,
                modulePositions[3].distanceMeters
            });
        Logger.recordOutput(getName() + "/state", getState().toString());
    }

    public void zeroOdometry() {
        swerveDrive.resetEncoders();
        swerveDrive.zeroHeading();
        swerveDrive.resetOdometry(new Pose2d());
    }

    private void updateAutoSequence() {
        boolean autoRequested = m_autoSequence != null && m_autoSequence.getAsBoolean();

        if (!autoRequested) {
            m_autoRunning = false;
            m_autoFinishedWhileHeld = false;
            m_autoOverrideActive = false;
            m_autoStage = 0;
            return;
        }

        if (m_autoFinishedWhileHeld) {
            m_autoOverrideActive = false;
            return;
        }

        if (!m_autoRunning) {
            startAutoSequence();
        }

        if (m_autoRunning) {
            runAutoSequence();
        }
    }

    private void startAutoSequence() {
        Pose2d current = swerveDrive.getPose();
        Pose2d[] candidates = Constants.AZ_RAMP_POSA_CANDIDATES;
        double bestDistance = Double.MAX_VALUE;
        int bestIndex = 0;
        for (int i = 0; i < candidates.length; i++) {
            double dist = current.getTranslation().getDistance(candidates[i].getTranslation());
            if (dist < bestDistance) {
                bestDistance = dist;
                bestIndex = i;
            }
        }

        switch (bestIndex) {
            case 0 -> {
                m_autoTargets[0] = Constants.AZ_rampRed1_posa;
                m_autoTargets[1] = Constants.AZ_rampRed1_posb;
                m_autoTargets[2] = Constants.AZ_rampRed1_posc;
            }
            case 1 -> {
                m_autoTargets[0] = Constants.AZ_rampRed2_posa;
                m_autoTargets[1] = Constants.AZ_rampRed2_posb;
                m_autoTargets[2] = Constants.AZ_rampRed2_posc;
            }
            case 2 -> {
                m_autoTargets[0] = Constants.AZ_rampBlue1_posa;
                m_autoTargets[1] = Constants.AZ_rampBlue1_posb;
                m_autoTargets[2] = Constants.AZ_rampBlue1_posc;
            }
            case 3 -> {
                m_autoTargets[0] = Constants.AZ_rampBlue2_posa;
                m_autoTargets[1] = Constants.AZ_rampBlue2_posb;
                m_autoTargets[2] = Constants.AZ_rampBlue2_posc;
            }
        }

        m_autoStage = 0;
        m_autoRunning = true;
        m_autoOverrideActive = true;
        beginAutoStage();
    }

    private void beginAutoStage() {
        m_autoStageStartPose = swerveDrive.getPose();
        m_autoStageTimeoutStart = Timer.getFPGATimestamp();
        m_autoLeftY = 0.0;
        m_autoLeftX = 0.0;
        m_autoRightX = 0.0;
        m_autoOverrideActive = true;
    }

    private void runAutoSequence() {
        Pose2d current = swerveDrive.getPose();
        Pose2d targetPose = m_autoTargets[m_autoStage];
        double targetHeading = targetPose.getRotation().getRadians();

        double dx = targetPose.getX() - current.getX();
        double dy = targetPose.getY() - current.getY();
        double translationKp = 1000.0;
        double rotationKp = 2.5;

        double maxStageSpeed =
            AUTO_STAGE_MAX_SPEED_MPS[Math.min(m_autoStage, AUTO_STAGE_MAX_SPEED_MPS.length - 1)];

        double headingError = MathUtil.angleModulus(targetHeading - current.getRotation().getRadians());
        double rotationCmd =
            MathUtil.clamp(
                headingError * rotationKp,
                -Constants.DriveConstants.kMaxAngularSpeed,
                Constants.DriveConstants.kMaxAngularSpeed);

        double forwardCmd =
            MathUtil.clamp(
                dx * translationKp,
                -maxStageSpeed,
                maxStageSpeed);
        double strafeCmd =
            MathUtil.clamp(
                dy * translationKp,
                -maxStageSpeed,
                maxStageSpeed);

        boolean positionReached = Math.abs(dx) < AUTO_POSITION_TOLERANCE && Math.abs(dy) < AUTO_POSITION_TOLERANCE;
        boolean headingReached = Math.abs(headingError) < AUTO_HEADING_TOLERANCE_RAD;
        boolean timedOut = (Timer.getFPGATimestamp() - m_autoStageTimeoutStart) > AUTO_STAGE_TIMEOUT_SEC;
        boolean crossedLine = hasCrossedProgressionLine(targetPose);

        if ((positionReached && headingReached) || timedOut || crossedLine) {
            m_autoStage++;
            if (m_autoStage >= m_autoTargets.length) {
                m_autoRunning = false;
                m_autoOverrideActive = false;
                m_autoFinishedWhileHeld = true;
                return;
            }
            beginAutoStage();
            return;
        }

        m_autoLeftY = forwardCmd;
        m_autoLeftX = strafeCmd;
        m_autoRightX = rotationCmd;
        m_autoOverrideActive = true;
    }

    private boolean hasCrossedProgressionLine(Pose2d target) {
        Pose2d current = swerveDrive.getPose();
        double dxStart = target.getX() - m_autoStageStartPose.getX();
        double dyStart = target.getY() - m_autoStageStartPose.getY();
        boolean useXAsPrimary = Math.abs(dxStart) >= Math.abs(dyStart);
        double offset = AUTO_STAGE_LINE_OFFSET_METERS;

        if (useXAsPrimary) {
            double approachDir = Math.signum(dxStart == 0.0 ? 1.0 : dxStart);
            double lineX = target.getX() - (offset * approachDir);
            return approachDir > 0
                ? current.getX() >= lineX
                : current.getX() <= lineX;
        }

        double approachDir = Math.signum(dyStart == 0.0 ? 1.0 : dyStart);
        double lineY = target.getY() - (offset * approachDir);
        return approachDir > 0
            ? current.getY() >= lineY
            : current.getY() <= lineY;
    }

    public void zeroGyro() {
        swerveDrive.zeroHeading();
    }

    public void offset180() {
        swerveDrive.offset180();
    }

    public void close() {}
}

//henry wuz here
