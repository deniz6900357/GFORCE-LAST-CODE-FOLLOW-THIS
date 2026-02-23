// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.FullSubsystem;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

// Use TimedRobot in simulation to avoid AdvantageKit Conduit native library issues
// Use LoggedRobot on real robot for full AdvantageKit functionality

public class Robot extends LoggedRobot{
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", "Mk5n-otonom");
    Logger.recordMetadata("RobotName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("GitDirty", BuildConstants.DIRTY ? "true" : "false");

    // Set up data receivers & replay source
    if (isReal()) {
      // Real robot - log to file and NetworkTables
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());

      // Enable power distribution logging
      new PowerDistribution(1, ModuleType.kRev);

    } else {
      // Simulation - minimal logging to avoid Conduit native library issues
      // NetworkTables only, no file logging
      Logger.addDataReceiver(new NT4Publisher());
    }

    // Start AdvantageKit logger (skip in simulation as we use TimedRobot)
    if (isReal()) {
      Logger.start();
    }

    m_robotContainer = new RobotContainer();
  }

  /**
   * Returns true if hardware alerts should be shown.
   * Used by subsystems to determine if disconnection alerts should be displayed.
   */
  public static boolean showHardwareAlerts() {
    return true; // Always show alerts (can be enhanced later with replay detection)
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Run periodicAfterScheduler for all FullSubsystems
    FullSubsystem.runAllPeriodicAfterScheduler();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Set initial pose based on alliance (alliance info is available now)
    m_robotContainer.setInitialPoseForAlliance();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
