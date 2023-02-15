// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utilities.RobotInformation;
import frc.robot.utilities.SwerveModuleConstants;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.MechanismSimulator;
import frc.robot.subsystems.LEDsubsystem;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIORobot;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorIORobot;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.rotateintake.PitchIntakeIORobot;
import frc.robot.subsystems.rotateintake.PitchIntakeIOSim;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;
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
import frc.robot.commands.MonitorPitchIntakeCommand;
import frc.robot.commands.PitchIntakeCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.autos.AutoCommandManager.subNames;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ExtendIntakeMotorSubsystem;
import frc.robot.subsystems.IntakeRollerMotorSubsystem;
import frc.robot.commands.TravelToTarget;
import frc.robot.commands.LEDCommand.LedPatterns;
import frc.robot.commands.armcommands.RunManipulatorRollerCommand;
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

    //Intake Motors
    private final ExtendIntakeMotorSubsystem m_ExtendIntakeMotorSubsystem = new ExtendIntakeMotorSubsystem(12);
    private final IntakeRollerMotorSubsystem m_IntakeRollerMotorSubsystem = new IntakeRollerMotorSubsystem(7);
    private final boolean isCompetitionRobot = true; //RobotInformation.queryIfCompetitionRobot(false);

    // Which Robot code should we use? competition or not
    //Cannot use an ID of 0
    //Changed the turningMotorID and cancoderID from 0 to 3
    //https://buildmedia.readthedocs.org/media/pdf/phoenix-documentation/latest/phoenix-documentation.pdf
    //page 100
    RobotInformation robotInfo = 
      (isCompetitionRobot) ?
        // Competition robot attributes
        new RobotInformation(true,
          new SwerveModuleConstants(8, 9, 9, 200.479),
          new SwerveModuleConstants(11, 10, 10, 11.338),
          new SwerveModuleConstants(1, 3, 3, 108.193  ),
          new SwerveModuleConstants(18, 19, 19, 117.158  ))
        :
        // Non-Competition robot attributes
        new RobotInformation(false,
          new SwerveModuleConstants(8, 9, 9, 114.69),
          new SwerveModuleConstants(11, 10, 10, 235.1),
          new SwerveModuleConstants(1, 3, 3, 84.28),
          new SwerveModuleConstants(18, 19, 19, 9.75));
   
 /* Modules */
  public final SwerveModuleConstants frontLeftModule = robotInfo.getFrontLeft();
  public final SwerveModuleConstants frontRightModule =  robotInfo.getFrontRight();
  public final SwerveModuleConstants backLeftModule = robotInfo.getBackLeft();
  public final SwerveModuleConstants backRightModule = robotInfo.getBackRight();
  
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
  CommandXboxController m_codriverController = new CommandXboxController(kCodriverControllerPort);


  // Subsystems \\
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final SwerveDrive m_robotDrive = new SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);
  private final PitchIntakeSubsystem m_PitchIntakeSubsystem = new PitchIntakeSubsystem(Robot.isReal()? new PitchIntakeIORobot(14): new PitchIntakeIOSim());
  
  private final TravelToTarget m_travelToTarget = new TravelToTarget( new Pose2d(3, 4, new Rotation2d(0)), m_robotDrive);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(Robot.isReal() ? new ArmIORobot(5) : new ArmIOSim());
  private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem(Robot.isReal() ? new ManipulatorIORobot(4, 15) : new ManipulatorIOSim());
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(Robot.isReal() ? new ElevatorIORobot(6) : new ElevatorIOSim());

  private final MechanismSimulator m_mechanismSimulator = new MechanismSimulator(m_armSubsystem, m_elevatorSubsystem, m_manipulatorSubsystem, m_PitchIntakeSubsystem, m_robotDrive);

  private final LEDsubsystem m_LEDsubsystem = new LEDsubsystem(0, 1,2,3 );
  // Commands \\
  private final RotateCommand m_rotateCommand = new RotateCommand(new Pose2d( 8.2423, 4.0513, new Rotation2d(0.0)), m_robotDrive);
  private final AutoBalanceCommand m_autoBalanceCommand = new AutoBalanceCommand(m_robotDrive);
  private final ExtendIntakeCommand m_ExtendIntakeCommand = new ExtendIntakeCommand(-6, m_ExtendIntakeMotorSubsystem);
  private final ExtendIntakeCommand m_RetractIntakeCommand = new ExtendIntakeCommand(6, m_ExtendIntakeMotorSubsystem);
  private final IntakeRollerCommand m_IntakeRoller = new IntakeRollerCommand(2, m_IntakeRollerMotorSubsystem);
  private final IntakeRollerCommand m_EjectRoller = new IntakeRollerCommand(-2, m_IntakeRollerMotorSubsystem);
  private final PitchIntakeCommand m_HighPitchIntakeCommand = new PitchIntakeCommand(90.0);
  private final PitchIntakeCommand m_MediumPitchIntakeCommand = new PitchIntakeCommand(0.0);
  private final PitchIntakeCommand m_LowPitchIntakeCommand = new PitchIntakeCommand(-90.0);

    //TODO REMOVE
    private static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    private static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    private static final double kMaxAccelerationMetersPerSecondSquared = 3;
    private static final double kPXController = 1;
    private static final double kPYController = 1;
  
    
  private AutoCommandManager m_autoManager;
  private Map<String, Command> eventCommandMap = new HashMap<>();

  private final SetArmDegreesCommand m_HighArmPosition = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, ArmSubsystem.highPosition, ManipulatorSubsystem.highPosition);
  private final SetArmDegreesCommand m_MediumArmPosition = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, ArmSubsystem.mediumPosition, ManipulatorSubsystem.mediumPosition);
  private final SetArmDegreesCommand m_GroundArmPosition = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, ArmSubsystem.groundPosition, ManipulatorSubsystem.groundPosition);
  private final SetArmDegreesCommand m_IntakeArmPosition = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, ArmSubsystem.intakePosition, ManipulatorSubsystem.intakePosition);
  private final SetArmDegreesCommand m_StowArmPosition = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, ArmSubsystem.stowPosition, ManipulatorSubsystem.stowPosition);
  private final SetArmDegreesCommand m_ArmMoveTest = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, -15, 0); // A
  private final SetArmDegreesCommand m_ManipulatorMoveTest = new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 45, 0); // B

  private final ElevatorMoveCommand m_HighestElevatorPosition = new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(36.0));
  private final ElevatorMoveCommand m_HighElevatorPosition = new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(22.64));
  private final ElevatorMoveCommand m_MedElevatorPosition = new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(11.32));
  private final ElevatorMoveCommand m_LowElevatorPosition = new ElevatorMoveCommand(m_elevatorSubsystem, 0);

  private final RunManipulatorRollerCommand m_ManipulatorRollerCommand = new RunManipulatorRollerCommand(m_manipulatorSubsystem, 0.5);
  private final RunManipulatorRollerCommand m_ManipulatorRollerStopCommand = new RunManipulatorRollerCommand(m_manipulatorSubsystem, 0.15);
  private final RunManipulatorRollerCommand m_ManipulatorRollerReleaseCommand = new RunManipulatorRollerCommand(m_manipulatorSubsystem, -0.3);
  private final RunManipulatorRollerCommand m_ManipulatorRollerShootCommand = new RunManipulatorRollerCommand(m_manipulatorSubsystem, -1);

  private final LEDCommand m_RunDisabledLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.DISABLED);
  private final LEDCommand m_RunConeRequestLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.CONEREQUEST);
  private final LEDCommand m_RunCubeRequestLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.CUBEREQUEST);
  private final LEDCommand m_RunConeAquiredLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.CONEAQUIRED);
  private final LEDCommand m_RunCubeAquiredLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.CUBEAQUIRED);
  private final LEDCommand m_RunBlueAllianceLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.BLUEALLIANCE);
  private final LEDCommand m_RunRedAllianceLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.REDALLIANCE);
  private final LEDCommand m_RunTeamColorsLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.TEAMCOLORS);
  private final LEDCommand m_RunRandomLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.RANDOMLED);
  private final LEDCommand m_RunAutoBalanceLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.AUTOBALANCE);

  public static final int kDriverControllerPort = 0;
  public static final int kCodriverControllerPort = 1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Auto Commands

    // TODO Add markers for real commands/paths
    eventCommandMap.put("marker1", new PrintCommand("Marker1Start********************"));
    eventCommandMap.put("marker2", new PrintCommand("Marker1End********************"));
    eventCommandMap.put("PreloadConeScore", new PrintCommand("NeedCommandForPreloadedConeScore"));
    eventCommandMap.put("Picks up an new cone or cube", new PrintCommand("NeedCommandforPickingupGamepiece"));
    eventCommandMap.put("Change of velocity", new PrintCommand("Need command to change velocity"));
    eventCommandMap.put("AutoBalance here", new PrintCommand("Need command to AutoBalance"));
    m_autoManager = new AutoCommandManager();
    m_autoManager.addSubsystem(subNames.SwerveDriveSubsystem, m_robotDrive);
    m_autoManager.initCommands(eventCommandMap);

    // Configure the button bindings
    configureButtonBindings();
    m_driverController.x().whileTrue(m_travelToTarget);
    m_driverController.y().whileTrue(m_rotateCommand);
    m_driverController.b().whileTrue(m_autoBalanceCommand);
    m_driverController.povUp().onTrue(m_HighPitchIntakeCommand);
    m_driverController.povRight().onTrue(m_MediumPitchIntakeCommand);
    m_driverController.povDown().onTrue(m_LowPitchIntakeCommand);
    m_driverController.leftBumper().whileTrue(m_HighElevatorPosition);
    m_driverController.rightBumper().whileTrue(m_MedElevatorPosition);
    m_driverController.a().whileTrue(m_LowElevatorPosition);
    m_driverController.back().whileTrue(m_HighestElevatorPosition);

    m_codriverController.x().whileTrue(m_HighArmPosition);
    m_codriverController.y().whileTrue(m_MediumArmPosition);
    //m_codriverController.a().whileTrue(m_GroundArmPosition);
    //m_codriverController.b().whileTrue(m_IntakeArmPosition);
    m_codriverController.a().whileTrue(m_ArmMoveTest);
    m_codriverController.b().whileTrue(m_ManipulatorMoveTest);
    //m_codriverController.rightBumper().whileTrue(m_StowArmPosition);
    m_codriverController.leftBumper().whileTrue(m_ManipulatorRollerCommand);
    m_codriverController.rightBumper().whileTrue(m_ManipulatorRollerStopCommand);
    m_codriverController.rightTrigger().whileTrue(m_ManipulatorRollerShootCommand);
    m_codriverController.leftTrigger().whileTrue(m_ManipulatorRollerReleaseCommand);
    // Configure default commands
    m_robotDrive.setDefaultCommand(new TeleopSwerve(m_robotDrive, m_driverController, translationAxis, strafeAxis, rotationAxis, true, true));
    m_fieldSim.initSim();
    m_ExtendIntakeMotorSubsystem.setDefaultCommand(m_RetractIntakeCommand);
    m_PitchIntakeSubsystem.setDefaultCommand(new MonitorPitchIntakeCommand(m_PitchIntakeSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    m_codriverController.rightTrigger().whileTrue(m_ExtendIntakeCommand);
    m_codriverController.povLeft().whileTrue(m_EjectRoller);
    m_codriverController.povDown().whileTrue(m_IntakeRoller);

  }

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
