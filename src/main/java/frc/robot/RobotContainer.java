// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.OdometryUtility;
import frc.robot.utilities.SwerveModuleConstants;
import frc.robot.utilities.vision.estimation.CameraProperties;
import frc.robot.utilities.vision.estimation.PNPResults;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import frc.robot.autos.AutoCommandManager;
import frc.robot.autos.TaxiOneBall;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.autos.AutoCommandManager.subNames;
import frc.robot.commands.TeleopSwerve;

import frc.robot.commands.TravelToTarget;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.commands.armcommands.MoveArmCommand;
public class RobotContainer {
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

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

  
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);


  // Subsystems \\
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final SwerveDrive m_robotDrive = new SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);
  
  private final TravelToTarget m_travelToTarget = new TravelToTarget( new Pose2d(3, 4, new Rotation2d(0)), m_robotDrive);
  private final ArmSubsystem m_arm = new ArmSubsystem(4, 5);

  // Commands \\
  private final RotateCommand m_rotateCommand = new RotateCommand(new Pose2d( 8.2423, 4.0513, new Rotation2d(0.0)), m_robotDrive);
  private final AutoBalanceCommand m_autoBalanceCommand = new AutoBalanceCommand(m_robotDrive);
  private AutoCommandManager m_autoManager;

  private final MoveArmCommand m_HighArmPosition = new MoveArmCommand(m_arm, 0.5, ArmSubsystem.highWristPosition, ArmSubsystem.highArmPosition);
  private final MoveArmCommand m_MediumArmPosition = new MoveArmCommand(m_arm, 0.5, ArmSubsystem.mediumWristPosition, ArmSubsystem.mediumArmPosition);
  private final MoveArmCommand m_GroundArmPosition = new MoveArmCommand(m_arm, 0.5, ArmSubsystem.groundWristPosition, ArmSubsystem.groundArmPosition);
  private final MoveArmCommand m_IntakeArmPosition = new MoveArmCommand(m_arm, 0.5, ArmSubsystem.intakeWristPosition, ArmSubsystem.intakeArmPosition);
  private final MoveArmCommand m_StowArmPosition = new MoveArmCommand(m_arm, 0.5, ArmSubsystem.stowWristPosition, ArmSubsystem.stowArmPosition);

  public static final int kDriverControllerPort = 0;
  //TODO REMOVE
  private static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  private static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  private static final double kMaxAccelerationMetersPerSecondSquared = 3;
  private static final double kPXController = 1;
  private static final double kPYController = 1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //TODO add port forwarding
    m_autoManager = new AutoCommandManager();
    m_autoManager.addSubsystem(subNames.SwerveDriveSubsystem, m_robotDrive);
    m_autoManager.initCommands();

    // Configure the button bindings
    configureButtonBindings();
    m_driverController.x().whileTrue(m_travelToTarget);
    m_driverController.y().whileTrue(m_rotateCommand);
    m_driverController.b().whileTrue(m_autoBalanceCommand);
    // Configure default commands
    m_robotDrive.setDefaultCommand(new TeleopSwerve(m_robotDrive, m_driverController, translationAxis, strafeAxis, rotationAxis, true, true));
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

    return m_autoManager.getAutonomousCommand();
    //TODO determine if autoManager needs to have andThen(() -> m_robotDrive.drive(0, 0, 0, false,false));

    //return new TaxiOneBall(m_robotDrive).andThen(() -> m_robotDrive.drive(0, 0, 0, false,false));

  }

  public void periodic() {
    m_fieldSim.periodic();
  }

  public void disabledInit() {
    // THis appears to cause robot angle to shift each time enable
    //m_robotDrive.resetAngleToAbsolute();
  }
    

  

}
