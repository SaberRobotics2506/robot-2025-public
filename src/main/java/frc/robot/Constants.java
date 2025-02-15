package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Constants {
    public static final double EXTAKE_MOTOR_SPEED = .4;

    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double TrueMaxSpeed = TunerConstants.kSpeedAt12VoltsChassis.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxSpeedPath = TunerConstants.kSpeedAt12VoltsPath.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    // Feeder Poses
    public static final Pose2d blueFeeder12Pose = new Pose2d(0.8, 0.7, Rotation2d.fromDegrees(45));
    public static final Pose2d blueFeeder13Pose = new Pose2d(0, 0, Rotation2d.fromDegrees(225));

    public static final Pose2d redFeeder1Pose = new Pose2d(16.1, 0.7, Rotation2d.fromDegrees(135));
    public static final Pose2d redFeeder2Pose = new Pose2d(16.1, 7.2, Rotation2d.fromDegrees(315));

    // Blue Reef Poses
    public static final Pose2d blue18Pose = new Pose2d(2.6, 4, Rotation2d.fromDegrees(0));
    public static final Pose2d blue17Pose = new Pose2d(3.4, 3, Rotation2d.fromDegrees(60));
    public static final Pose2d blue22Pose = new Pose2d(4.6, 3, Rotation2d.fromDegrees(300));
    public static final Pose2d blue21Pose = new Pose2d(5.16, 4, Rotation2d.fromDegrees(0));
    public static final Pose2d blue20Pose = new Pose2d(4.45, 5, Rotation2d.fromDegrees(240));
    public static final Pose2d blue19Pose = new Pose2d(3.2, 5, Rotation2d.fromDegrees(120));

    // Red Reef Poses
    public static final Pose2d red7Pose = new Pose2d(13.75, 4, Rotation2d.fromDegrees(180));
    public static final Pose2d red8Pose = new Pose2d(13.27 ,5, Rotation2d.fromDegrees(240));
    public static final Pose2d red9Pose = new Pose2d(11.9, 5, Rotation2d.fromDegrees(300));
    public static final Pose2d red10Pose = new Pose2d(11.3, 4, Rotation2d.fromDegrees(0));
    public static final Pose2d red11Pose = new Pose2d(11.9, 3, Rotation2d.fromDegrees(60));
    public static final Pose2d red6Pose = new Pose2d(13.1, 3, Rotation2d.fromDegrees(120));

    // Processor Poses
    public static final Pose2d blueProcessorPose = new Pose2d(0, 0, Rotation2d.fromDegrees(270));
    public static final Pose2d redProcessorPose = new Pose2d(11.1, 8.55, Rotation2d.fromDegrees(90));

    // Barge Poses
    public static final Pose2d blueBargePose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
    public static final Pose2d redBargePose = new Pose2d(9.1, 2.1, Rotation2d.fromDegrees(0));

    // Pathplaner constraints
    public static final PathConstraints constraints = new PathConstraints(
        MaxSpeedPath,
        2.5, 
        MaxAngularRate,
        Units.degreesToRadians(720)
    );

    public static class Commands {

        // Feeder Paths
        public static Command feeder1Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(redFeeder1Pose, constraints, 0);
            } else {
                return AutoBuilder.pathfindToPose(blueFeeder12Pose, constraints);
            }
        }
        public static Command feeder2Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(redFeeder2Pose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blueFeeder13Pose, constraints);
            }
        }

        // Reef Paths
        public static Command reef3Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(red11Pose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blue20Pose, constraints);
            }
        }
        public static Command reef4Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(red10Pose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blue21Pose, constraints);
            }
        }
        public static Command reef5Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(red9Pose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blue22Pose, constraints);
            }
        }
        public static Command reef6Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(red8Pose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blue17Pose, constraints);
            }
        }
        public static Command reef7Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(red7Pose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blue18Pose, constraints);
            }
        }
        public static Command reef8Path() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(red6Pose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blue19Pose, constraints);
            }
        }

        // Processor Path
        public static Command processorPath() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(redProcessorPose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blueProcessorPose, constraints);
            }
        }

        // Barge Path
        public static Command bargePath() {
            if (CommandSwerveDrivetrain.onRed()) {
                return AutoBuilder.pathfindToPose(redBargePose, constraints);
            } else {
                return AutoBuilder.pathfindToPose(blueBargePose, constraints);
            }
        }
    }   
}

