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
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.partimodu.LED;
import edu.wpi.first.wpilibj.Joystick;
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
    private final Joystick climbJoystick = new Joystick(1);

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

    // Climb subsystem (NEO motors ID 41 ve 42)
    private final Climb climb = new Climb();

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

        // Otonom atış komutu: AimToHub + Hood + Flywheel + 0.5s sonra Feeder, toplam 4 saniye
        NamedCommands.registerCommand("Shoot",
            Commands.parallel(
                new AimToHubCommand(
                    drivetrain,
                    () -> 0.0,
                    () -> 0.0,
                    MaxSpeed,
                    MaxAngularRate
                ),
                hood.runTrackTargetCommand(),
                flywheel.runTrackTargetCommand(),
                Commands.waitSeconds(0.5)
                    .andThen(Commands.startEnd(
                        feeder::feed,
                        feeder::stop,
                        feeder
                    ))
            ).withTimeout(4.0).withName("Auto: Shoot")
        );

        // Otonom intake komutu: 5 saniye çalışıp otomatik durur
        // defer() ile her seferinde yeni komut oluşturulur - tekrar çalışma sorunu olmaz
        NamedCommands.registerCommand("Intake",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(0.9),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(5.0).withName("Auto: Intake 5s"),
                java.util.Set.of(intakeMotor)
            )
        );

        NamedCommands.registerCommand("Intake2",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(0.9),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(2.0).withName("Auto: Intake 2s"),
                java.util.Set.of(intakeMotor)
            )
        );
        NamedCommands.registerCommand("Intake3",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(0.9),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(3.0).withName("Auto: Intake 3s"),
                java.util.Set.of(intakeMotor)
            )
        );
        NamedCommands.registerCommand("Intake4",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(0.9),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(4.0).withName("Auto: Intake 4s"),
                java.util.Set.of(intakeMotor)
            )
        );
        NamedCommands.registerCommand("Intake5",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(0.9),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(5.0).withName("Auto: Intake 5s"),
                java.util.Set.of(intakeMotor)
            )
        );

        // Intake durdurma komutu
        NamedCommands.registerCommand("StopIntake",
            Commands.defer(
                () -> intakeMotor.stopCommand().withName("Auto: Stop Intake"),
                java.util.Set.of(intakeMotor)
            )
        );
    }

    private void configureBindings() {
        // Varsayılan olarak LED'ler düz mavi yansın
        ledSubsystem.setDefaultCommand(ledSubsystem.run(ledSubsystem::setHeartbeatFastBlue).withName("LED: Default Heartbeat Fast Blue"));

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

        // POV Left: AutoTrench Sol Geçit
        joystick.povLeft().whileTrue(
            AutoTrench.allianceAwareLeftTrenchSequence(drivetrain)
        );

        // POV Right: AutoTrench Sağ Geçit
        joystick.povRight().whileTrue(
            AutoTrench.allianceAwareRightTrenchSequence(drivetrain)
        );

        // Right Bumper (RB): AutoTrench Sağ Geçit
        // Basılı tut = çalışır, bırak = durur
        joystick.button(8).whileTrue(
            AutoTrench.allianceAwareRightTrenchSequence(drivetrain)
        );

        // Left Bumper (LB): AutoTrench Sol Geçit
        // Basılı tut = çalışır, bırak = durur
        joystick.button(7).whileTrue(
            AutoTrench.allianceAwareLeftTrenchSequence(drivetrain)
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

        // B button: AutoTrench Sol test
        joystick.b().whileTrue(
            AutoTrench.allianceAwareLeftTrenchSequence(drivetrain)
        );

        // Hood zero etme (Start butonu) - İLK ÇALIŞTIRMADA MUTLAKA YAPIN!
        joystick.start().onTrue(hood.zeroCommand());

        // POV Up: Auto-align to tower scoring position (odometry-based)
        // Y > 3.75 → (1.5, 4.0), Y <= 3.75 → (1.5, 3.4)
        // Limelight zaten arka planda odometry'yi düzeltiyor
        joystick.povUp().whileTrue(AutoAlign.alignToTower(drivetrain));
        joystick.povDown().whileTrue(
            Commands.parallel(
                hood.setAngleCommand(Math.toRadians(15.0)),
                feeder.reverseFirstTwoCommand(),
                Commands.print("🔽 POV DOWN: Hood 15° (MIN) açıya gidiyor...")
            ).withName("Hood: Test MIN 15°")
        );

        // X tuşu: Gyro reset
        joystick.x().onTrue(
            Commands.runOnce(() -> {
                drivetrain.seedFieldCentric(new Rotation2d());
                System.out.println("GYRO RESET! Yeni yon: 0");
            }).ignoringDisable(true)
        );

        // RT (sağ trigger): Koşullu atış
        // X > 5.25 ise: sabit noktaya aim + hood 0° + flywheel %80 (296 rad/s) + feeder
        // X <= 5.25 ise: normal AimToHub + Hood Track + Flywheel Track + Feeder
        joystick.rightTrigger().whileTrue(
            Commands.defer(() -> {
                Pose2d robotPose = drivetrain.getState().Pose;
                if (robotPose.getX() > 5.25) {
                    // Far shot mode - Y'ye göre hedef belirle
                    Translation2d target = robotPose.getY() > 4.035
                        ? new Translation2d(2.0, 6.0)
                        : new Translation2d(2.0, 2.0);
                    System.out.println("RT: Far shot mode - hedef: " + target);

                    return Commands.parallel(
                        // Belirli noktaya aim
                        drivetrain.applyRequest(() -> {
                            Pose2d pose = drivetrain.getState().Pose;
                            Translation2d toTarget = target.minus(pose.getTranslation());
                            Rotation2d angleToTarget = new Rotation2d(toTarget.getX(), toTarget.getY());
                            double error = AimToHubCommand.getModuloRotation(
                                angleToTarget.getDegrees() - pose.getRotation().getDegrees()
                            );
                            double steering = 0.08 * error;
                            if (Math.abs(error) > 1.0) {
                                steering += Math.signum(error) * 0.02;
                            }
                            steering = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, steering));
                            return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                .withRotationalRate(steering);
                        }),
                        hood.setAngleCommand(0.0), // Hood 0°
                        flywheel.runVelocityCommand(351.5), // %95 hız (0.95 * 370)
                        Commands.waitSeconds(0.5)
                            .andThen(Commands.startEnd(feeder::feed, feeder::stop, feeder))
                    ).withName("RT: Far Shot");
                } else {
                    // Normal shooting mode
                    return Commands.parallel(
                        new AimToHubCommand(drivetrain, joystick::getLeftY, joystick::getLeftX, MaxSpeed, MaxAngularRate),
                        hood.runTrackTargetCommand(),
                        flywheel.runTrackTargetCommand(),
                        Commands.waitSeconds(0.5)
                            .andThen(Commands.startEnd(feeder::feed, feeder::stop, feeder))
                    ).withName("RT: Aim + Hood + Flywheel + Feeder");
                }
            }, java.util.Set.of(drivetrain, hood, flywheel, feeder))
        );

        // LT (sol trigger): Intake - basılı tut, bırak durur
        joystick.leftTrigger().whileTrue(
                Commands.parallel(
                        intakeMotor.intakeCommand(),
                        ledSubsystem.run(ledSubsystem::setBlinkWhite).withName("LED: Blink White")
                ).withName("LT: Intake"));

        // X tuşu (Klavye - Simülasyon): AutoTrench test için
        // Simülasyonda klavyeden X'e basarak test edebilirsiniz - SAĞ trench (SMART direction)
        GenericHID keyboard = new GenericHID(0);
        new Trigger(() -> keyboard.getRawButton(27)) // X tuşu (button 27)
            .onTrue(AutoTrench.allianceAwareRightTrenchSequence(drivetrain));

        // Climb joystick (kanal 1) sol stick aşağı (axis 1 < -0.1) → climb motorları çalışır
        new Trigger(() -> climbJoystick.getRawAxis(1) < -0.1)
            .whileTrue(Commands.run(() -> climb.setSpeed(-climbJoystick.getRawAxis(1)), climb)
                .finallyDo(climb::stop));

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