// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilTagEstimator;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Extake;
import frc.robot.subsystems.LED;

import frc.robot.commands.ExtakePiece;
import frc.robot.commands.ExtakeReverse;
import edu.wpi.first.wpilibj.DigitalInput;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(Constants.MaxSpeed);

    public static final CommandXboxController joystick = new CommandXboxController(0);
    public static final CommandXboxController buttonBoard = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public TalonSRX m_extakeMotor = new TalonSRX(21);
    public DigitalInput m_irSensor = new DigitalInput(0);
    public final Extake m_extake = new Extake(m_extakeMotor, m_irSensor);

    public final LED m_leds = new LED(m_extake);

    public final AprilTagEstimator aprilTagEstimator = new AprilTagEstimator(drivetrain);
    // public LED m_leds = new LED(m_extake);

    public RobotContainer() {

        NamedCommands.registerCommand("ExtakeCoral", new ExtakePiece(m_extake));

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * 0.7 * Constants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * 0.7 * Constants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * 0.7 * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
            
        // Turbo mode
        joystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Constants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.a().onTrue(new ExtakePiece(m_extake));

        // Cycle command
        // joystick.x().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        // joystick.x().whileTrue(cycleCommand.repeatedly());

        // pathfinding
        buttonBoard.a().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.a().whileTrue(new DeferredCommand(() -> Constants.Commands.feeder1Path(), Constants.Commands.feeder1Path().getRequirements()));

        buttonBoard.b().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.b().whileTrue(new DeferredCommand(() -> Constants.Commands.feeder2Path(), Constants.Commands.feeder2Path().getRequirements()));

        buttonBoard.start().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.start().whileTrue(new DeferredCommand(() -> Constants.Commands.reef8Path(), Constants.Commands.reef8Path().getRequirements()).andThen(new ExtakePiece(m_extake)));

        buttonBoard.back().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.back().whileTrue(new DeferredCommand(() -> Constants.Commands.reef7Path(), Constants.Commands.reef7Path().getRequirements()).andThen(new ExtakePiece(m_extake)));

        buttonBoard.rightBumper().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.rightBumper().whileTrue(new DeferredCommand(() -> Constants.Commands.reef6Path(), Constants.Commands.reef6Path().getRequirements()).andThen(new ExtakePiece(m_extake)));

        buttonBoard.leftBumper().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.leftBumper().whileTrue(new DeferredCommand(() -> Constants.Commands.reef5Path(), Constants.Commands.reef5Path().getRequirements()).andThen(new ExtakePiece(m_extake)));

        buttonBoard.y().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.y().whileTrue(new DeferredCommand(() -> Constants.Commands.reef4Path(), Constants.Commands.reef4Path().getRequirements()).andThen(new ExtakePiece(m_extake)));

        buttonBoard.x().whileTrue(new InstantCommand(() -> m_leds.pathfinding = true)).whileFalse(new InstantCommand(() -> m_leds.pathfinding = false));
        buttonBoard.x().whileTrue(new DeferredCommand(() -> Constants.Commands.reef3Path(), Constants.Commands.reef3Path().getRequirements()).andThen(new ExtakePiece(m_extake)));
    }

    public enum CyclingCommands {
        REEF,
        FEEDER
    }

    public CyclingCommands selectCycleCommand() {
        if (!m_extake.getIrSensor()) {
            return CyclingCommands.REEF;
        } else {
            return CyclingCommands.FEEDER;
        }
    }

    public Command cycleCommand =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(CyclingCommands.REEF, new DeferredCommand(() -> Constants.Commands.reef7Path(), Constants.Commands.reef7Path().getRequirements()).andThen(new ExtakePiece(m_extake))),
                Map.entry(CyclingCommands.FEEDER, new DeferredCommand(() -> Constants.Commands.feeder1Path(), Constants.Commands.feeder1Path().getRequirements()))),
            this::selectCycleCommand);
            
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
