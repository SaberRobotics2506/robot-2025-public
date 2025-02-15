package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.commands.ExtakePiece;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    private static final double ROBOT_MASS_KG = 33.5;
    private static final double ROBOT_MOI = 3.84;
    private static final double WHEEL_COF = 1.0;

    private static final RobotConfig PP_CONFIG =
    new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            0.050,
            Constants.TrueMaxSpeed,
            WHEEL_COF,
            DCMotor.getFalcon500(1).withReduction(6.75),
            TunerConstants.FrontLeft.SlipCurrent,
            1),
        getModuleTranslations());

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    private BaseStatusSignal[] signals;

    // public PoseEstimator poseEstimator;

    public Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();

    private final Field2d field2d = new Field2d();

    Rotation2d currAngle = new Rotation2d(Math.PI/180 * pigeon2Subsystem.getPigeonYaw(true));

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("swerve");

    public StringPublisher currentCommand;
    public DoublePublisher swerveSpeed0;
    public DoublePublisher swerveSpeed1;
    public DoublePublisher swerveSpeed2;
    public DoublePublisher swerveSpeed3;
    public DoublePublisher swerveAngle0;
    public DoublePublisher swerveAngle1;
    public DoublePublisher swerveAngle2;
    public DoublePublisher swerveAngle3;
    public DoublePublisher poseX;
    public DoublePublisher poseY;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(15).per(Second),        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();

        currentCommand = table.getStringTopic("currentCommand").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed0").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed1").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed2").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed3").publish();
        swerveAngle0 = table.getDoubleTopic("swerveAngles/angle0").publish();
        swerveAngle1 = table.getDoubleTopic("swerveAngles/angle1").publish();
        swerveAngle2 = table.getDoubleTopic("swerveAngles/angle2").publish();
        swerveAngle3 = table.getDoubleTopic("swerveAngles/angle3").publish();
        poseX = table.getDoubleTopic("poseX").publish();
        poseY = table.getDoubleTopic("poseY").publish();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();

        currentCommand = table.getStringTopic("currentCommand").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed0").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed1").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed2").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed3").publish();
        swerveAngle0 = table.getDoubleTopic("swerveAngles/angle0").publish();
        swerveAngle1 = table.getDoubleTopic("swerveAngles/angle1").publish();
        swerveAngle2 = table.getDoubleTopic("swerveAngles/angle2").publish();
        swerveAngle3 = table.getDoubleTopic("swerveAngles/angle3").publish();
        poseX = table.getDoubleTopic("poseX").publish();
        poseY = table.getDoubleTopic("poseY").publish();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();

        currentCommand = table.getStringTopic("currentCommand").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed0").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed1").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed2").publish();
        swerveSpeed0 = table.getDoubleTopic("swerveSpeeds/speed3").publish();
        swerveAngle0 = table.getDoubleTopic("swerveAngles/angle0").publish();
        swerveAngle1 = table.getDoubleTopic("swerveAngles/angle1").publish();
        swerveAngle2 = table.getDoubleTopic("swerveAngles/angle2").publish();
        swerveAngle3 = table.getDoubleTopic("swerveAngles/angle3").publish();
        poseX = table.getDoubleTopic("poseX").publish();
        poseY = table.getDoubleTopic("poseY").publish();
    }

    private void configureAutoBuilder() {
        AutoBuilder.configure(
            () -> getState().Pose,   // Supplier of current robot pose
            this::resetPose,         // Consumer for seeding pose against auto
            () -> getState().Speeds, // Supplier of current robot speeds
            // Consumer of ChassisSpeeds and feedforwards to drive the robot
            (speeds, feedforwards) -> setControl(
                m_pathApplyRobotSpeeds.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
            new PPHolonomicDriveController(
                // PID constants for translation
                new PIDConstants(1.05, 0, 0.005),
                // PID constants for rotation
                new PIDConstants(3, 0, 0)
            ),
            PP_CONFIG,
            // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this // Subsystem for requirements
        );
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("MaxSpeed", Constants.MaxSpeed);

        // currentCommand.set(getCurrentCommand().getName());

        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("SwerveSpeed" + i, getState().ModuleStates[i].speedMetersPerSecond);
            SmartDashboard.putNumber("SwerveAngle" + i, getState().ModuleStates[i].angle.getDegrees());
        }

        // swerveSpeed0.set(getState().ModuleStates[0].speedMetersPerSecond);
        // swerveSpeed1.set(getState().ModuleStates[1].speedMetersPerSecond);
        // swerveSpeed2.set(getState().ModuleStates[2].speedMetersPerSecond);
        // swerveSpeed3.set(getState().ModuleStates[3].speedMetersPerSecond);
        // swerveAngle0.set(getState().ModuleStates[0].angle.getDegrees());
        // swerveAngle1.set(getState().ModuleStates[1].angle.getDegrees());
        // swerveAngle2.set(getState().ModuleStates[2].angle.getDegrees());
        // swerveAngle3.set(getState().ModuleStates[3].angle.getDegrees());

        SmartDashboard.putNumber("PoseX", getState().Pose.getX());
        SmartDashboard.putNumber("PoseY", getState().Pose.getY());

        poseX.set(getState().Pose.getX());
        poseY.set(getState().Pose.getY());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    public BaseStatusSignal[] getSignals() {
        return signals;
    }

    public static boolean onRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDev) {
        super.setVisionMeasurementStdDevs(visionMeasurementStdDev);
    }

    public Pose2d getPose() {
        return super.getState().Pose;
    }
}
