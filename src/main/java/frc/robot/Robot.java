// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import org.littletonrobotics.junction.LogFileUtil;
// Advantage Kit
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utilities.OdometryUtility;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private static final int REAL = 0;

  private static final int SIM = 1;

  private static final int REPLAY = 2;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // Allow pyshical Photonvison cammera to be used while simulating on PC
    // https://docs.photonvision.org/en/latest/docs/programming/photonlib/hardware-in-the-loop-sim.html
    // NOTE Requires Photolib camera to run network server!!!! But this will break on setting is on when used on robot
    if(RobotBase.isSimulation() && OdometryUtility.CONNECTED_PHOTOVISION_CAMERA) {
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      inst.stopServer();
      // Change the IP address in the below function to the IP address you use to connect to the PhotonVision UI.
      inst.setServer(OdometryUtility.PHOTOVISION_NETWORK_SERVER); // photonvision.local
      inst.startClient4("Robot Simulation");
   }


    //Advantage Kit
    Logger logger = Logger.getInstance();
    //TODO setUseTiming(Constants.getMode() != Mode.REPLAY);
    logger.recordMetadata("Robot", "FRCRobot");
    logger.recordMetadata("TuningMode", Boolean.toString(false));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }
    int mode = SIM;
    mode = Robot.isReal()?REAL:SIM;
    switch (mode) {
      case REAL:
        String folder = "/media/sda1/";
        folder = "/home/lvuser";
        if (folder != null) {
          logger.addDataReceiver(new WPILOGWriter(folder));
        // } else {
          //TODO logNoFileAlert.set(true);
        }
        logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance();
        break;
      case SIM:
        logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(path));
        logger.addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }
    logger.start();

    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //Needed for simulation
    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.disabledInit();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
     m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testExit(){
      // TODO m_robotContaner.testExit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
