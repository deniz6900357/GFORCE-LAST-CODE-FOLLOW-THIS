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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.GenericHID;

import frc.robot.automation.AutoAlign;
import frc.robot.automation.AutoTrench;
import frc.robot.commands.AimToHubCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.IntakeMotor;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOReal;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOReal;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.partimodu.LED;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // 5% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // Use velocity control for drive motors
            .withSteerRequestType(com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo); // Advanced
                                                                                                           // steer
                                                                                                           // control
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Intake subsystem (single Kraken X60 motor on CANivore)
    private final IntakeMotor intakeMotor = new IntakeMotor();

    // Shooter subsystems (MA architecture - separate subsystems)
    // NOTE: Simulation implementations removed - only real hardware supported
    private final Flywheel flywheel = new Flywheel("Flywheel",
        RobotBase.isReal() ? new FlywheelIOReal() : new FlywheelIO() {});
    private final Hood hood = new Hood(
        RobotBase.isReal() ? new HoodIOReal() : new HoodIO() {}); // Hood motor: Kraken X44 (ID 34) on CANivore

    // Feeder subsystem (NEO motor with 27:1 reduction)
    // Use sim implementation in simulation mode to avoid REVLib driver dependency
    private final Feeder feeder = new Feeder(RobotBase.isReal() ? new FeederIOReal() : new FeederIOSim());

    // LED Subsystem
    private final LED ledSubsystem = new LED();

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

        // REMOVED: Default commands for Hood/Flywheel (safety issue - motors spinning on boot)
        // Hood ve Flywheel sadece manuel olarak B tuşu ile çalıştırılır

        System.out.println("✅ RobotContainer initialized - No default commands for shooter!");

        // Note: Initial pose will be set in teleopInit() when alliance is available
    }

    /**
     * Sets the initial pose based on alliance. Call this in teleopInit() or
     * autonomousInit().
     */
    public void setInitialPoseForAlliance() {
        // Set initial pose based on alliance for AdvantageScope visualization
        // Blue: Near blue hub (X: 4.6m, Y: 2.35m), facing forward (0°)
        // Red: Near red hub (X: 11.9m, Y: 2.35m), facing blue wall (180°)
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue);

        // Gyro'nun mevcut yönünü al
        Rotation2d currentRotation = drivetrain.getState().Pose.getRotation();

        Pose2d startPose;
        if (alliance == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
            // Blue alliance: Mevcut rotasyonu koru (gyro'yu bozmadan)
            startPose = new Pose2d(4.6, 2.35, currentRotation);
            System.out.println("Blue Alliance - Robot initialized at position: " + startPose);
        } else {
            // Red alliance: Mevcut rotasyonu koru
            startPose = new Pose2d(11.9, 2.35, currentRotation);
            System.out.println("Red Alliance - Robot initialized at position: " + startPose);
        }
        drivetrain.resetPose(startPose);

        // Field-centric direction'ı alliance'a göre ayarla (operator perspective)
        // Bu sadece joystick input'larının referans frame'ini değiştirir, gyro'yu etkilemez
        System.out.println("✅ Initial pose set. Gyro preserved at: " + currentRotation.getDegrees() + "°");
    }

    /**
     * Register named commands that can be used in PathPlanner autonomous routines.
     * These commands can be referenced by name in the PathPlanner GUI.
     */
    private void registerNamedCommands() {
        // Register AimToHub command for autonomous use
        // Usage: Add "AimToHub" event marker in PathPlanner
        // Command will finish automatically when aligned with hub (within 3° tolerance
        // for 5 loops)
        NamedCommands.registerCommand("AimToHub", new AimToHubCommand(
                drivetrain,
                () -> 0.0, // No forward movement in auto
                () -> 0.0, // No strafe movement in auto
                MaxSpeed,
                MaxAngularRate,
                true // finishWhenAtTarget = true for autonomous
        ));
    }

    private void configureBindings() {
        // Varsayılan olarak LED'ler düz mavi yansın
        ledSubsystem.setDefaultCommand(ledSubsystem.run(ledSubsystem::setSolidBlue).withName("LED: Default Blue"));

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // POV Left: Point wheels (eski B tuşu)
        joystick.povLeft().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Right Bumper (RB - Button 8): AutoTrench Sağ Geçit
        // Blue: Alt geçit (Y: 0.65), Red: Üst geçit (Y: 7.45, mirrored)
        // SMART: X < 4.0 ise ters yönden gider (3.5 -> 5.7)
        // BASILI TUTUNCA: Çalışır, bırakınca durur (onTrue başlatır, onFalse iptal eder)
        joystick.button(8).onTrue(
            Commands.runOnce(() -> {
                System.out.println("⚡ RB TUŞUNA BASILDI (Button 8) - AutoTrench SAĞ tetikleniyor...");
                SmartDashboard.putBoolean("AutoTrench/RightRunning", true);
            })
            .andThen(AutoTrench.allianceAwareRightTrenchSequence(drivetrain))
            .finallyDo(() -> {
                System.out.println("⏹️ RB BIRAKILDI - AutoTrench SAĞ durduruluyor...");
                SmartDashboard.putBoolean("AutoTrench/RightRunning", false);
            })
        );

        // RB bırakıldığında: Drivetrain'in şu anki komutunu iptal et (AutoTrench durur)
        joystick.button(8).onFalse(
            Commands.runOnce(() -> {
                System.out.println("🛑 RB BIRAKILDI - Drivetrain komutları iptal ediliyor...");
                if (drivetrain.getCurrentCommand() != null) {
                    drivetrain.getCurrentCommand().cancel();
                }
            })
        );

        // Left Bumper (LB - Button 7): AutoTrench Sol Geçit
        // Blue: Üst geçit (Y: 7.45), Red: Alt geçit (Y: 0.65, mirrored)
        // SMART: X < 4.0 ise ters yönden gider (3.5 -> 5.7)
        // BASILI TUTUNCA: Çalışır, bırakınca durur (onTrue başlatır, onFalse iptal eder)
        joystick.button(7).onTrue(
            Commands.runOnce(() -> {
                System.out.println("⚡ LB TUŞUNA BASILDI (Button 7) - AutoTrench SOL tetikleniyor...");
                SmartDashboard.putBoolean("AutoTrench/LeftRunning", true);
            })
            .andThen(AutoTrench.allianceAwareLeftTrenchSequence(drivetrain))
            .finallyDo(() -> {
                System.out.println("⏹️ LB BIRAKILDI - AutoTrench SOL durduruluyor...");
                SmartDashboard.putBoolean("AutoTrench/LeftRunning", false);
            })
        );

        // LB bırakıldığında: Drivetrain'in şu anki komutunu iptal et (AutoTrench durur)
        joystick.button(7).onFalse(
            Commands.runOnce(() -> {
                System.out.println("🛑 LB BIRAKILDI - Drivetrain komutları iptal ediliyor...");
                if (drivetrain.getCurrentCommand() != null) {
                    drivetrain.getCurrentCommand().cancel();
                }
            })
        );

        // A button: Auto-aim + Hood tracking (Hub'a döner VE hood açısı ayarlanır)
        // Basılı tuttuğun sürece: Hub'a döner VE hood açısı ayarlanır (PARALEL)
        final AimToHubCommand aimCommand = new AimToHubCommand(
            drivetrain,
            joystick::getLeftY,
            joystick::getLeftX,
            MaxSpeed,
            MaxAngularRate
        );

        joystick.a().whileTrue(
            Commands.parallel(
                aimCommand,
                hood.runTrackTargetCommand()
            ).withName("A: Aim + Hood Track")
        );

        // B button: Hood only (Predictive shot) - TESTING
        // Basılı tuttuğun sürece: Hood açısı ayarlanır (Flywheel disabled for testing)
        joystick.b().whileTrue(
            hood.runTrackTargetCommand().withName("B: Hood Only (Track Target)")
        );

        // Hood zero etme (POV sağ veya Start butonu) - İLK ÇALIŞTIRMADA MUTLAKA YAPIN!
        joystick.povRight().onTrue(hood.zeroCommand());
        joystick.start().onTrue(hood.zeroCommand());

        // POV Up: Auto-align to tower scoring position (odometry-based)
        // Y > 3.75 → (1.5, 4.0), Y <= 3.75 → (1.5, 3.4)
        // Limelight zaten arka planda odometry'yi düzeltiyor
        joystick.povUp().whileTrue(AutoAlign.alignToTower(drivetrain));
        joystick.povDown().whileTrue(
            Commands.parallel(
                hood.setAngleCommand(Math.toRadians(15.0)),
                Commands.print("🔽 POV DOWN: Hood 15° (MIN) açıya gidiyor...")
            ).withName("Hood: Test MIN 15°")
        );

        // X tuşu: Gyro reset - Robot'un mevcut yönünü 0° olarak ayarla
        joystick.x().onTrue(
            Commands.runOnce(() -> {
                // Gyro'yu 0° olarak ayarla (forward = 0°)
                drivetrain.seedFieldCentric(new Rotation2d());
                System.out.println("🧭 GYRO RESET! Yeni yön: 0°");
            }).ignoringDisable(true)
        );

        // RT (sağ trigger): Feeder - basılı tut, feeder çalışır; bırak, feeder durur
        joystick.rightTrigger().whileTrue(
                Commands.parallel(
                        Commands.startEnd(
                                feeder::feed,
                                feeder::stop,
                                feeder),
                        ledSubsystem.run(ledSubsystem::setBlinkBlue).withName("LED: Blink Blue") // Atış tuşuna
                                                                                                 // basılıyken Mavi
                                                                                                 // Yanıp Sönme
                ));

        // LT (sol trigger): Intake - basılı tut, Kraken X60 motor çalışır; bırak, motor
        // durur
        joystick.leftTrigger().whileTrue(
                Commands.parallel(
                        intakeMotor.intakeCommand(), // Uses INTAKE_SPEED constant (30%)
                        ledSubsystem.run(ledSubsystem::setBlinkWhite).withName("LED: Blink White") // Intake tuşuna
                                                                                                   // basılıyken Beyaz
                                                                                                   // Yanıp Sönme
                ));

        // X tuşu (Klavye - Simülasyon): AutoTrench test için
        // Simülasyonda klavyeden X'e basarak test edebilirsiniz - SAĞ trench (SMART direction)
        GenericHID keyboard = new GenericHID(0);
        new Trigger(() -> keyboard.getRawButton(27)) // X tuşu (button 27)
            .onTrue(AutoTrench.allianceAwareRightTrenchSequence(drivetrain));

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