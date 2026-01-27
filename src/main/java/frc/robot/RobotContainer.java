// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.AimToHubCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeMotor;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOReal;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 5% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // Use velocity control for drive motors
            .withSteerRequestType(com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo); // Advanced steer control
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Intake subsystem (single Kraken X60 motor on CANivore)
    private final IntakeMotor intakeMotor = new IntakeMotor();

    // Shooter subsystems (MA architecture - separate subsystems)
    private final Flywheel flywheel = new Flywheel(new FlywheelIOReal());
    private final Hood hood = new Hood(new HoodIOSim()); // Hood hardware not connected - using simulation

    // Feeder subsystem (NEO motor with 27:1 reduction)
    private final Feeder feeder = new Feeder(new FeederIOReal());

    // Auto chooser for selecting autonomous routines
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register named commands for PathPlanner BEFORE building auto chooser
        registerNamedCommands();

        configureBindings();

        // Build auto chooser using PathPlanner's AutoBuilder
        // This will show all autos in the deploy/pathplanner/autos folder
        autoChooser = AutoBuilder.buildAutoChooser();

        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Note: Initial pose will be set in teleopInit() when alliance is available
    }

    /**
     * Sets the initial pose based on alliance. Call this in teleopInit() or autonomousInit().
     */
    public void setInitialPoseForAlliance() {
        // Set initial pose based on alliance for AdvantageScope visualization
        // Blue: Near blue hub (X: 4.6m, Y: 2.35m), facing forward (0°)
        // Red: Near red hub (X: 11.9m, Y: 2.35m), facing blue wall (180°)
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance().orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);
        Pose2d startPose;
        if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
            startPose = new Pose2d(4.6, 2.35, Rotation2d.kZero);
            System.out.println("Blue Alliance - Robot initialized at position: " + startPose);
        } else {
            startPose = new Pose2d(11.9, 2.35, Rotation2d.fromDegrees(180));
            System.out.println("Red Alliance - Robot initialized at position (facing blue wall): " + startPose);
        }
        drivetrain.resetPose(startPose);
    }

    /**
     * Register named commands that can be used in PathPlanner autonomous routines.
     * These commands can be referenced by name in the PathPlanner GUI.
     */
    private void registerNamedCommands() {
        // Register AimToHub command for autonomous use
        // Usage: Add "AimToHub" event marker in PathPlanner
        // Command will finish automatically when aligned with hub (within 3° tolerance for 5 loops)
        NamedCommands.registerCommand("AimToHub", new AimToHubCommand(
            drivetrain,
            () -> 0.0,        // No forward movement in auto
            () -> 0.0,        // No strafe movement in auto
            MaxSpeed,
            MaxAngularRate,
            true              // finishWhenAtTarget = true for autonomous
        ));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // reset the field-centric heading on left bumper press
        // Resets gyro to 0 degrees (toward red alliance wall) for consistent controls
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
            var currentPose = drivetrain.getState().Pose;
            drivetrain.resetPose(new Pose2d(currentPose.getTranslation(), Rotation2d.kZero));
        }));

        // brake on right bumper
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // Hood zero etme (POV sağ) - İLK ÇALIŞTIRMADA MUTLAKA YAPIN!
        joystick.povRight().onTrue(hood.zeroCommand());

        // RT (sağ trigger): Feeder - basılı tut, feeder çalışır; bırak, feeder durur
        joystick.rightTrigger().whileTrue(
            Commands.startEnd(
                feeder::feed,
                feeder::stop,
                feeder
            )
        );

        // LT (sol trigger): Intake - basılı tut, Kraken X60 motor çalışır; bırak, motor durur
        joystick.leftTrigger().whileTrue(
            Commands.startEnd(
                () -> intakeMotor.setSpeed(0.1),
                intakeMotor::stop,
                intakeMotor
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Return the selected auto from the chooser
        Command selectedAuto = autoChooser.getSelected();

        // Debug: Print which auto was selected
        if (selectedAuto != null) {
            System.out.println("Starting autonomous: " + selectedAuto.getName());
        } else {
            System.out.println("WARNING: No autonomous command selected!");
        }

        return selectedAuto;
    }
}
