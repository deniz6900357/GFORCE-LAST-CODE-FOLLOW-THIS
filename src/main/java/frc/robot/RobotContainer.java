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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.commands.ApproachToTagCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // Use velocity control for drive motors
            .withSteerRequestType(com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo); // Advanced steer control
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

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
    }

    /**
     * Register named commands that can be used in PathPlanner autonomous routines.
     * These commands can be referenced by name in the PathPlanner GUI.
     */
    private void registerNamedCommands() {
        // Register ApproachToTag command for autonomous use
        // Usage: Add "ApproachToTag" event marker in PathPlanner
        // Note: In autonomous, there's no joystick input so robot will only auto-aim without moving
        NamedCommands.registerCommand("ApproachToTag", new ApproachToTagCommand(
            drivetrain,
            () -> 0.0,        // No forward movement in auto
            () -> 0.0,        // No strafe movement in auto
            MaxSpeed,
            MaxAngularRate
        ));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with X (left)
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
        // Resets gyro to alliance-appropriate forward direction
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> {
            var alliance = DriverStation.getAlliance();
            Rotation2d newRotation = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red
                ? Rotation2d.k180deg  // Red alliance faces blue wall
                : Rotation2d.kZero;   // Blue alliance faces red wall
            var currentPose = drivetrain.getState().Pose;
            drivetrain.resetPose(new Pose2d(currentPose.getTranslation(), newRotation));
        }));

        // brake on right bumper
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

        // Approach AprilTag with Limelight - hold A button
        // Robot will automatically aim at AprilTag while driver retains movement control
        joystick.a().whileTrue(new ApproachToTagCommand(
            drivetrain,
            joystick::getLeftY,   // Forward/backward control
            joystick::getLeftX,   // Strafe left/right control
            MaxSpeed,             // Max translational speed
            MaxAngularRate        // Max rotational speed
        ));


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
