// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utilities.SwerveModuleConstants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.MechanismSimulator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIORobot;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIORobot;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.HashMap;
import java.util.Map;


import frc.robot.autos.AutoCommandManager;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.autos.AutoCommandManager.subNames;
import frc.robot.commands.TeleopSwerve;

import frc.robot.commands.TravelToTarget;

import frc.robot.commands.armcommands.SetArmDegreesCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
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
  CommandXboxController m_codriverController = new CommandXboxController(kCodriverControllerPort);


  // Subsystems \\
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final SwerveDrive m_robotDrive = new SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);
  
  private final TravelToTarget m_travelToTarget = new TravelToTarget( new Pose2d(3, 4, new Rotation2d(0)), m_robotDrive);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(Robot.isReal() ? new ArmIORobot(4, 5) : new ArmIOSim());
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(Robot.isReal() ? new ElevatorIORobot(6) : new ElevatorIOSim());

  private final MechanismSimulator m_mechanismSimulator = new MechanismSimulator(m_armSubsystem, m_elevatorSubsystem);

  // Commands \\
  private final RotateCommand m_rotateCommand = new RotateCommand(new Pose2d( 8.2423, 4.0513, new Rotation2d(0.0)), m_robotDrive);
  private final AutoBalanceCommand m_autoBalanceCommand = new AutoBalanceCommand(m_robotDrive);
  private AutoCommandManager m_autoManager;
  private Map<String, Command> eventCommandMap = new HashMap<>();

  private final SetArmDegreesCommand m_HighArmPosition = new SetArmDegreesCommand(m_armSubsystem, ArmSubsystem.highWristPosition, ArmSubsystem.highShoulderPosition);
  private final SetArmDegreesCommand m_MediumArmPosition = new SetArmDegreesCommand(m_armSubsystem, ArmSubsystem.mediumWristPosition, ArmSubsystem.mediumShoulderPosition);
  private final SetArmDegreesCommand m_GroundArmPosition = new SetArmDegreesCommand(m_armSubsystem, ArmSubsystem.groundWristPosition, ArmSubsystem.groundShoulderPosition);
  private final SetArmDegreesCommand m_IntakeArmPosition = new SetArmDegreesCommand(m_armSubsystem, ArmSubsystem.intakeWristPosition, ArmSubsystem.intakeShoulderPosition);
  private final SetArmDegreesCommand m_StowArmPosition = new SetArmDegreesCommand(m_armSubsystem, ArmSubsystem.stowWristPosition, ArmSubsystem.stowShoulderPosition);

  private final ElevatorMoveCommand m_HighElevatorPosition = new ElevatorMoveCommand(m_elevatorSubsystem, 22.64);
  private final ElevatorMoveCommand m_MedElevatorPosition = new ElevatorMoveCommand(m_elevatorSubsystem, 11.32);
  private final ElevatorMoveCommand m_LowElevatorPosition = new ElevatorMoveCommand(m_elevatorSubsystem, 0);
  public static final int kDriverControllerPort = 0;
  public static final int kCodriverControllerPort = 1;

  //TODO REMOVE
  private static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  private static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  private static final double kMaxAccelerationMetersPerSecondSquared = 3;
  private static final double kPXController = 1;
  private static final double kPYController = 1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Auto Commands

    // TODO Add markers for real commands/paths
    eventCommandMap.put("marker1", new PrintCommand("Marker1Start********************"));
    eventCommandMap.put("marker2", new PrintCommand("Marker1End********************"));
    m_autoManager = new AutoCommandManager();
    m_autoManager.addSubsystem(subNames.SwerveDriveSubsystem, m_robotDrive);
    m_autoManager.initCommands(eventCommandMap);

    // Configure the button bindings
    configureButtonBindings();
    m_driverController.x().whileTrue(m_travelToTarget);
    m_driverController.y().whileTrue(m_rotateCommand);
    m_driverController.b().whileTrue(m_autoBalanceCommand);
    m_driverController.leftBumper().whileTrue(m_HighElevatorPosition);
    m_driverController.rightBumper().whileTrue(m_MedElevatorPosition);
    m_driverController.a().whileTrue(m_LowElevatorPosition);

    m_codriverController.x().whileTrue(m_HighArmPosition);
    m_codriverController.y().whileTrue(m_MediumArmPosition);
    m_codriverController.a().whileTrue(m_GroundArmPosition);
    m_codriverController.b().whileTrue(m_IntakeArmPosition);
    m_codriverController.rightBumper().whileTrue(m_StowArmPosition);
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
    m_mechanismSimulator.periodic();
  }

  public void disabledInit() {
    // THis appears to cause robot angle to shift each time enable
    //m_robotDrive.resetAngleToAbsolute();
  }
    

  

}
