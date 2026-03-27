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
    private final CommandXboxController climbJoystick = new CommandXboxController(1);

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
            startPose = new Pose2d(4.6, 2.35, currentRotation);
            drivetrain.resetPose(startPose);
            drivetrain.seedFieldCentric(new Rotation2d());
            System.out.println("Blue Alliance - Robot initialized at position: " + startPose);
        } else {
            startPose = new Pose2d(11.9, 2.35, currentRotation);
            drivetrain.resetPose(startPose);
            drivetrain.seedFieldCentric(new Rotation2d());
            System.out.println("Red Alliance - Robot initialized at position: " + startPose);
        }

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
                    () -> intakeMotor.setDutyCycle(1.0),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(5.0).withName("Auto: Intake 5s"),
                java.util.Set.of(intakeMotor)
            )
        );

        NamedCommands.registerCommand("Intake2",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(1.0),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(2.0).withName("Auto: Intake 2s"),
                java.util.Set.of(intakeMotor)
            )
        );
        NamedCommands.registerCommand("Intake3",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(1.0),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(3.0).withName("Auto: Intake 3s"),
                java.util.Set.of(intakeMotor)
            )
        );
        NamedCommands.registerCommand("Intake4",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(1.0),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(4.0).withName("Auto: Intake 4s"),
                java.util.Set.of(intakeMotor)
            )
        );
        NamedCommands.registerCommand("Intake5",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(1.0),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(5.0).withName("Auto: Intake 5s"),
                java.util.Set.of(intakeMotor)
            )
        );

        NamedCommands.registerCommand("Intake75",
            Commands.defer(
                () -> Commands.startEnd(
                    () -> intakeMotor.setDutyCycle(1.0),
                    intakeMotor::stop,
                    intakeMotor
                ).withTimeout(7.5).withName("Auto: Intake 7.5s"),
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

        NamedCommands.registerCommand("ClimbUp",
            Commands.runOnce(() -> climb.setPosition(105), climb).withName("Auto: Climb Up")
        );
        NamedCommands.registerCommand("ClimbDown",
            Commands.runOnce(() -> climb.setPosition(15), climb).withName("Auto: Climb Down")
        );
    }

    private void configureBindings() {
        // Varsayılan olarak LED'ler düz mavi yansın
        ledSubsystem.setDefaultCommand(ledSubsystem.run(ledSubsystem::setBreathBlue).withName("LED: Default Breath Blue"));

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

        // RT (sağ trigger): Koşullu atış (Alliance-aware)
        // Blue Alliance: X > 4.6 → sabit noktaya aim + hood 0° + flywheel %90 + feeder
        //                X <= 4.6 → normal AimToHub + Hood/Flywheel Track + Feeder
        // Red Alliance:  X > 12 → sabit noktaya aim + hood 0° + flywheel %90 + feeder
        //                X <= 12 → hiçbir şey yapma
        joystick.rightTrigger().whileTrue(
            Commands.defer(() -> {
                Pose2d robotPose = drivetrain.getState().Pose;
                boolean isRed = edu.wpi.first.wpilibj.DriverStation.getAlliance()
                    .orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue)
                    == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

                if (isRed) {
                    if (robotPose.getX() < 12.0) {
                        // Red Alliance far shot - Y'ye göre hedef belirle (predictive)
                        Translation2d target = robotPose.getY() > 4.02
                            ? new Translation2d(14.54, 6.0)
                            : new Translation2d(14.54, 2.0);
                        System.out.println("RT: Red far shot - hedef: " + target);

                        return Commands.parallel(
                            drivetrain.applyRequest(() -> {
                                Rotation2d targetAngle = frc.robot.subsystems.shooter.LaunchCalculator
                                    .getInstance().getDriveAngleForTarget(target).plus(Rotation2d.k180deg);
                                double error = AimToHubCommand.getModuloRotation(
                                    targetAngle.getDegrees() - drivetrain.getState().Pose.getRotation().getDegrees()
                                );
                                double steering = 0.08 * error;
                                if (Math.abs(error) > 1.0) steering += Math.signum(error) * 0.02;
                                steering = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, steering));
                                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                    .withRotationalRate(steering);
                            }),
                            hood.setAngleCommand(0.0),
                            flywheel.runVelocityCommand(333.0),
                            Commands.waitSeconds(0.5)
                                .andThen(Commands.startEnd(feeder::feed, feeder::stop, feeder))
                        ).withName("RT: Red Far Shot");
                    } else {
                        // Red Alliance, X >= 12: AimToHub + Hood/Flywheel track (Blue normal moduyla aynı)
                        return Commands.parallel(
                            new AimToHubCommand(drivetrain, joystick::getLeftY, joystick::getLeftX, MaxSpeed, MaxAngularRate),
                            hood.runTrackTargetCommand(),
                            flywheel.runTrackTargetCommand(),
                            Commands.waitSeconds(0.5)
                                .andThen(Commands.startEnd(feeder::feed, feeder::stop, feeder))
                        ).withName("RT: Red Hub Shot");
                    }
                } else {
                    // Blue Alliance
                    if (robotPose.getX() > 4.6) {
                        // Blue far shot - Y'ye göre hedef belirle (predictive)
                        Translation2d target = robotPose.getY() > 4.02
                            ? new Translation2d(2.0, 6.0)
                            : new Translation2d(2.0, 2.0);
                        System.out.println("RT: Blue far shot - hedef: " + target);

                        return Commands.parallel(
                            drivetrain.applyRequest(() -> {
                                Rotation2d targetAngle = frc.robot.subsystems.shooter.LaunchCalculator
                                    .getInstance().getDriveAngleForTarget(target).plus(Rotation2d.k180deg);
                                double error = AimToHubCommand.getModuloRotation(
                                    targetAngle.getDegrees() - drivetrain.getState().Pose.getRotation().getDegrees()
                                );
                                double steering = 0.08 * error;
                                if (Math.abs(error) > 1.0) steering += Math.signum(error) * 0.02;
                                steering = Math.max(-MaxAngularRate, Math.min(MaxAngularRate, steering));
                                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                                    .withRotationalRate(steering);
                            }),
                            hood.setAngleCommand(0.0),
                            flywheel.runVelocityCommand(333.0),
                            Commands.waitSeconds(0.5)
                                .andThen(Commands.startEnd(feeder::feed, feeder::stop, feeder))
                        ).withName("RT: Blue Far Shot");
                    } else {
                        // Blue Alliance, X <= 4.6: normal AimToHub + track
                        return Commands.parallel(
                            new AimToHubCommand(drivetrain, joystick::getLeftY, joystick::getLeftX, MaxSpeed, MaxAngularRate),
                            hood.runTrackTargetCommand(),
                            flywheel.runTrackTargetCommand(),
                            Commands.waitSeconds(0.5)
                                .andThen(Commands.startEnd(feeder::feed, feeder::stop, feeder))
                        ).withName("RT: Aim + Hood + Flywheel + Feeder");
                    }
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

        // Climb controller (kanal 1): sol stick → motor 41, sağ stick → motor 42
        new Trigger(() -> Math.abs(climbJoystick.getLeftY()) > 0.1 || Math.abs(climbJoystick.getRightY()) > 0.1)
            .whileTrue(Commands.run(() -> climb.setSpeedIndependent(
                -climbJoystick.getLeftY(),
                -climbJoystick.getRightY()
            ), climb).finallyDo(climb::stop));

        // Sol trigger → intake tersine
        climbJoystick.leftTrigger().whileTrue(intakeMotor.intakeCommand(-100.0));

        // X tuşu → hood homing (hardstop'a gidip sıfırla)
        climbJoystick.x().onTrue(hood.zeroCommand());

        // POV Up → encoder 104'e git, POV Down → 30'a, POV Left → 0'a
        climbJoystick.povUp().onTrue(Commands.runOnce(() -> climb.setPosition(105), climb));
        climbJoystick.povDown().onTrue(Commands.runOnce(() -> climb.setPosition(15), climb));
        climbJoystick.povLeft().onTrue(Commands.runOnce(() -> climb.setPosition(0), climb));

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