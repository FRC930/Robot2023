// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utilities.SwerveModuleConstants;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


    /* Modules */
    //Cannot use an ID of 0
    //Changed the turningMotorID and cancoderID from 0 to 3
    public static final SwerveModuleConstants frontLeftModule = 
      new SwerveModuleConstants(8, 9, 9, 114.69);
    public static final SwerveModuleConstants frontRightModule = 
      new SwerveModuleConstants(11, 10, 10, 235.1);
    public static final SwerveModuleConstants backLeftModule = 
      new SwerveModuleConstants(1, 3, 3, 84.28);
    public static final SwerveModuleConstants backRightModule = 
      new SwerveModuleConstants(18, 19, 19, 9.75);
    //https://buildmedia.readthedocs.org/media/pdf/phoenix-documentation/latest/phoenix-documentation.pdf
    //page 100

  // The robot's subsystems
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final SwerveDrive m_robotDrive = new SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  
  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);

    public static final int kDriverControllerPort = 0;
    //TODO REMOVE
    private static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    private static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    private static final double kMaxAccelerationMetersPerSecondSquared = 3;
    private static final double kPXController = 1;
    private static final double kPYController = 1;
  
    

  // The driver's controller
  XboxController m_driverController = new XboxController(kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            // TODO switch SwerveDrive Command and joyswitch deadband control see REV example
            () ->
                m_robotDrive.drive(
                    m_driverController.getLeftY(), //Throttle, forward
                    m_driverController.getLeftX(), //Strafe, sideways
                    m_driverController.getRightX(), //Rotate, turn
                    true, true),
            m_robotDrive));
      // TODO this forgot line for simulation
      m_fieldSim.initSim();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                SwerveDrive.kMaxSpeedMetersPerSecond,
                kMaxAccelerationMetersPerSecondSquared) //TODO REMOVE? AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(SwerveDrive.getSwerveKinematics());

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
          kMaxAngularSpeedRadiansPerSecond, 0, 0, new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared));
            //TODO REMOVE? AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPoseMeters, // Functional interface to feed supplier
            SwerveDrive.getSwerveKinematics(),

            // Position controllers
            new PIDController(kPXController, 0, 0),//TODO REMOVE? AutoConstants.kPXController, 0, 0),
            new PIDController(kPYController, 0, 0),//TODO REMOVE? AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setSwerveModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false,false));
  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public void disabledInit() {
    // THis appears to cause robot angle to shift each time enable
    //m_robotDrive.resetAngleToAbsolute();
  }

}
