// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.AutoCommandManager;
import frc.robot.autos.AutoCommandManager.subNames;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.SwerveLockCommand;
import frc.robot.commands.LEDCommand.LedPatterns;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TravelToTarget;
import frc.robot.commands.armcommands.RunManipulatorRollerCommand;
import frc.robot.simulation.FieldSim;
import frc.robot.simulation.MechanismSimulator;
import frc.robot.subsystems.ExtendIntakeMotorSubsystem;
import frc.robot.subsystems.IntakeRollerMotorSubsystem;
import frc.robot.subsystems.LEDsubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIORobot;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorIORobot;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorIORobot;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.RobotInformation;
// import frc.robot.utilities.TimeOfFlightUtility;
import frc.robot.utilities.RobotInformation.WhichRobot;
import frc.robot.utilities.SwerveModuleConstants;
import frc.robot.utilities.TargetScorePositionUtility;
import frc.robot.utilities.TargetScorePositionUtility.Target;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  Alliance alliance = Alliance.Invalid;

  private final double INTAKE_EXTEND_VOLTAGE = 6.0;
  private final double INTAKE_ROLLER_VOLTAGE = 7.0;

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;   
  private final int rotationAxis = XboxController.Axis.kRightX.value;

    //Intake Motors
    private final ExtendIntakeMotorSubsystem m_ExtendIntakeMotorSubsystem = new ExtendIntakeMotorSubsystem(16);
    private final IntakeRollerMotorSubsystem m_IntakeRollerMotorSubsystem = new IntakeRollerMotorSubsystem(7);
    private final WhichRobot whichRobot = RobotInformation.queryWhichRobotUsingPreferences();

    //Desired Target
    private TargetScorePositionUtility m_targetScorePositionUtility = new TargetScorePositionUtility();

    // Which Robot code should we use? competition or not
    //Cannot use an ID of 0
    //Changed the turningMotorID and cancoderID from 0 to 3
    //https://buildmedia.readthedocs.org/media/pdf/phoenix-documentation/latest/phoenix-documentation.pdf
    //page 100
    RobotInformation robotInfo = 
     (whichRobot == WhichRobot.COMPETITION_ROBOT) ?
        // Competition robot attributes
        new RobotInformation(whichRobot,
          new SwerveModuleConstants(8, 9, 9, 198.896),
          new SwerveModuleConstants(11, 10, 10, 9.141),
          new SwerveModuleConstants(1, 3, 3, 102.217),
          new SwerveModuleConstants(18, 19, 19, 209.180))
        :
        // Non-Competition robot attributes
        new RobotInformation(whichRobot,
          new SwerveModuleConstants(8, 9, 9, 114.69),
          new SwerveModuleConstants(11, 10, 10, 235.1),
          new SwerveModuleConstants(1, 3, 3, 84.28),
          new SwerveModuleConstants(18, 19, 19, 9.75));

  public static final int kDriverControllerPort = 0;
  public static final int kCodriverControllerPort = 1;
   
 /* Modules */
  public final SwerveModuleConstants frontLeftModule = robotInfo.getFrontLeft();
  public final SwerveModuleConstants frontRightModule =  robotInfo.getFrontRight();
  public final SwerveModuleConstants backLeftModule = robotInfo.getBackLeft();
  public final SwerveModuleConstants backRightModule = robotInfo.getBackRight();
  
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(kDriverControllerPort);
  CommandXboxController m_codriverController = new CommandXboxController(kCodriverControllerPort);

  // Subsystems \\
  private final SwerveDrive m_robotDrive = new SwerveDrive(frontLeftModule, frontRightModule, backLeftModule, backRightModule);
  private final FieldSim m_fieldSim = new FieldSim(m_robotDrive);
  //private final PitchIntakeSubsystem m_PitchIntakeSubsystem = new PitchIntakeSubsystem(Robot.isReal()? new PitchIntakeIORobot(14): new PitchIntakeIOSim());
  
  private final TravelToTarget m_travelToTarget = new TravelToTarget( new Pose2d(3, 4, new Rotation2d(0)), m_robotDrive);
  private final ArmIO armio = Robot.isReal() ? new ArmIORobot(5) : new ArmIOSim();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(armio);
  private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem(Robot.isReal() ? new ManipulatorIORobot(4, 15, 17) : new ManipulatorIOSim());
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(Robot.isReal() ? new ElevatorIORobot(6, 12)  : new ElevatorIOSim());
  private final MechanismSimulator m_mechanismSimulator = new MechanismSimulator(m_armSubsystem, m_elevatorSubsystem, m_manipulatorSubsystem, /*m_PitchIntakeSubsystem,*/ m_robotDrive);
  private final LEDsubsystem m_LEDsubsystem = new LEDsubsystem(0, 1,2,3 );

  // Utilities \\
  // private final TimeOfFlightUtility m_timeOfFlight = new TimeOfFlightUtility(1);
  
  // Commands \\
  private Command m_highTargetCommand = 
    CommandFactoryUtility.createScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, false);

  private Command m_mediumTargetCommand =
    CommandFactoryUtility.createScoreMediumCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, false);
  
  private Command m_lowTargetCommand = 
    CommandFactoryUtility.createScoreLowCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, false);
  
  //private final RotateCommand m_rotateCommand = new RotateCommand(new Pose2d( 8.2423, 4.0513, new Rotation2d(0.0)), m_robotDrive);
  private final AutoBalanceCommand m_autoBalanceCommand = new AutoBalanceCommand(m_robotDrive);
  private final SwerveLockCommand m_SwerveLockCommand = new SwerveLockCommand(m_robotDrive, true);

  // private final ExtendIntakeCommand m_ExtendIntakeCommand = new ExtendIntakeCommand(-INTAKE_EXTEND_VOLTAGE, m_ExtendIntakeMotorSubsystem);
  // private final ExtendIntakeCommand m_RetractIntakeCommand = new ExtendIntakeCommand(INTAKE_EXTEND_VOLTAGE, m_ExtendIntakeMotorSubsystem);
  // private final IntakeRollerCommand m_IntakeRoller = new IntakeRollerCommand(-INTAKE_ROLLER_VOLTAGE, m_IntakeRollerMotorSubsystem);
  // private final IntakeRollerCommand m_EjectRoller = new IntakeRollerCommand(INTAKE_ROLLER_VOLTAGE, m_IntakeRollerMotorSubsystem);
  // private final PitchIntakeCommand m_HighPitchIntakeCommand = new PitchIntakeCommand(m_PitchIntakeSubsystem, 90.0);
  // private final PitchIntakeCommand m_LowPitchIntakeCommand = new PitchIntakeCommand(m_PitchIntakeSubsystem, -90.0);
  //private PitchIntakeCommand m_CurrentPitchIntakeCommand;

  private final TeleopSwerve m_TeleopSwerve = new TeleopSwerve(m_robotDrive, m_driverController, translationAxis, strafeAxis, rotationAxis, true, true);
    
  private AutoCommandManager m_autoManager;
  private Map<String, Command> eventCommandMap = new HashMap<>();

  private final LEDCommand m_RunLEDPattern = new LEDCommand(m_LEDsubsystem, LedPatterns.DISABLED);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // If not FMS controlled add to teleop init too (for practice match and Red/Blue alliance need to be correctly set)
    checkDSUpdate();

    // Auto Commands
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreHighCone", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreMidCone", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "armIntakeCone", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "stowArm", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "waitTilArmDown", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "intakeElevatorPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "intakeArmPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "intakeManipulatorPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreHighElevator", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreHighArm", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreHighManipulator", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreHighNoStow", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "backIntakeElevatorPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "backIntakeArmPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "backIntakeManipulatorPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "backCubeIntakeElevatorPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "backCubeIntakeArmPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "backCubeIntakeManipulatorPos", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem); 
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreHighManipulatorAndNotRelease", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "manipulatorHold", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreGroundCube", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "scoreMidCube", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    CommandFactoryUtility.addAutoCommandEvent(eventCommandMap, "backCubeIntake", 
        m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    

    //TODO remove
    // eventCommandMap = eventCommandMap = new HashMap<>();

    m_autoManager = new AutoCommandManager();
    m_autoManager.addSubsystem(subNames.SwerveDriveSubsystem, m_robotDrive);
    m_autoManager.addSubsystem(subNames.ElevatorSubsystem, m_elevatorSubsystem);
    m_autoManager.addSubsystem(subNames.ArmSubsystem, m_armSubsystem);
    m_autoManager.addSubsystem(subNames.ManipulatorSubsystem, m_manipulatorSubsystem);
    m_autoManager.initCommands(eventCommandMap);

    // Configure the button bindings
    configureButtonBindings();
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(m_TeleopSwerve);
    m_fieldSim.initSim();
    // m_ExtendIntakeMotorSubsystem.setDefaultCommand(m_RetractIntakeCommand);
    // m_PitchIntakeSubsystem.setDefaultCommand(new PitchIntakeCommand(m_PitchIntakeSubsystem, 0));
    //stow arm position as default
  }

  void checkDSUpdate() {
    Alliance currentAlliance = DriverStation.getAlliance();

    // If we have data, and have a new alliance from last time
    if (DriverStation.isDSAttached() && currentAlliance != alliance) {
      m_robotDrive.setOriginBasedOnAlliance();
      alliance = currentAlliance;
    }

    Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/currentAlliance", currentAlliance.toString());
  } 
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
 
  private void configureButtonBindings() {

    SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/DriverController", DriverStation.getJoystickIsXbox(0));
    SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/Co-DriverController", DriverStation.getJoystickIsXbox(1));
    //Final Button Bindings
    //--DRIVER CONTROLLER--//
    //.and() makes it so both buttons must be held in order to run the command
    // TODO how are we planning on moving the arm based on the target score position utility

    m_driverController.y().onTrue(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED))
      .onFalse(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
        .andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)));

    // m_driverController.b().and(m_driverController.rightBumper().negate())
    //   .onTrue(new ExtendIntakeCommand(-INTAKE_EXTEND_VOLTAGE, m_ExtendIntakeMotorSubsystem).withTimeout(0.5).andThen(new IntakeRollerCommand(INTAKE_ROLLER_VOLTAGE, m_IntakeRollerMotorSubsystem)))
    //   .onFalse(new ExtendIntakeCommand(INTAKE_EXTEND_VOLTAGE, m_ExtendIntakeMotorSubsystem));

    m_driverController.povUp().toggleOnTrue(new InstantCommand(()->armio.adjustOffsetDegrees(15.0)));
    m_driverController.povDown().toggleOnTrue(new InstantCommand(()->armio.adjustOffsetDegrees(-15.0)));

    m_codriverController.rightBumper()
      .onTrue(CommandFactoryUtility.createArmBackCubeIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem))
      .onFalse(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));

    // Intakes from ground
    m_driverController.leftBumper()
      .whileTrue(CommandFactoryUtility.createArmIntakeLowCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem))
      .onFalse(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));

    m_driverController.rightTrigger()
      .onTrue(
        new ConditionalCommand(m_highTargetCommand, 
          new ConditionalCommand(
            m_mediumTargetCommand, 
            m_lowTargetCommand, 
            m_targetScorePositionUtility::isMedium), 
          m_targetScorePositionUtility::isHigh)
      )
      .onFalse(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));

    // Intakes from substation
    m_driverController.rightBumper().and(m_driverController.b().negate())
      .whileTrue(CommandFactoryUtility.createSingleSubstationCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem))
      .onFalse(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));
    
    m_driverController.x().and(m_driverController.b().negate())
      .whileTrue(CommandFactoryUtility.createDoubleSubstationCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem))
      .onFalse(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));

    // Slow drive
    m_driverController.leftStick().onTrue(new InstantCommand(() -> m_TeleopSwerve.toggleSpeed()));
    
    //Auto balance
    m_driverController.start().whileTrue(m_autoBalanceCommand);
  
    //Lock wheels in an X position
    m_driverController.a().toggleOnTrue(m_SwerveLockCommand);

    //--CODRIVER CONTROLLER--//
    // Arm intake
    // m_codriverController.leftBumper().whileTrue(m_EjectRoller); // Eject intake button

    // Will eventually be Intake Handoff (Intake from bottom gives game bject to top)
    // m_codriverController.leftTrigger().onTrue()

    // Back Cone Pickup
    m_codriverController.leftTrigger().onTrue(CommandFactoryUtility.createArmBackIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem))
      .onFalse(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));

    m_driverController.leftTrigger().onTrue(CommandFactoryUtility.createGroundIntakeExtendCommand(m_ExtendIntakeMotorSubsystem, m_IntakeRollerMotorSubsystem, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem))
      .onFalse(CommandFactoryUtility.createGroundIntakeRetractCommand(m_ExtendIntakeMotorSubsystem, m_IntakeRollerMotorSubsystem, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));
  
       m_codriverController.a().onTrue(CommandFactoryUtility.createExtendIntakeCommand(m_ExtendIntakeMotorSubsystem, m_IntakeRollerMotorSubsystem, false))
      .onFalse(CommandFactoryUtility.createRetractIntakeCommand(m_ExtendIntakeMotorSubsystem, m_IntakeRollerMotorSubsystem));


    //Arm positions
    m_codriverController.povUp().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.high));
    m_codriverController.povLeft().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.medium));
    m_codriverController.povRight().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.medium));
    m_codriverController.povDown().toggleOnTrue(m_targetScorePositionUtility.setDesiredTargetCommand(Target.low));
  
    //Cube and Cone selector
    m_codriverController.b().onTrue(new InstantCommand(() -> m_RunLEDPattern.toggleConeCubePattern()));
    // OLD method required to request cone/cube (but required user to hold)
    // m_codriverController.b().onTrue(new InstantCommand(() -> m_RunLEDPattern.setPattern(LedPatterns.CONEREQUEST)))
    //   .onFalse(new InstantCommand(() -> m_RunLEDPattern.setPattern(LedPatterns.CUBEREQUEST)));

    // TODO REMOVE this is not how determine it xboxcontroller is working!!
    // // Trigger indicator
    // m_driverController.leftTrigger()
    //   .onTrue(new PutToSmartDashboardCommand("DriverController/LeftTrigger", true))
    //   .onFalse(new PutToSmartDashboardCommand("DriverController/LeftTrigger", false));
    // m_driverController.rightTrigger()
    //   .onTrue(new PutToSmartDashboardCommand("DriverController/RightTrigger", true))
    //   .onFalse(new PutToSmartDashboardCommand("DriverController/RightTrigger", false));
    // m_codriverController.leftTrigger()
    //   .onTrue(new PutToSmartDashboardCommand("CodriverController/LeftTrigger", true))
    //   .onFalse(new PutToSmartDashboardCommand("CodriverController/LeftTrigger", false));
    // m_codriverController.rightTrigger()
    //   .onTrue(new PutToSmartDashboardCommand("CodriverController/RightTrigger", true))
    //   .onFalse(new PutToSmartDashboardCommand("CodriverController/RightTrigger", false));

    //m_codriverController.x().whileTrue(new AutoBalanceCommand(m_robotDrive, false));
    //m_codriverController.y().whileTrue(new AutoBalanceCommand(m_robotDrive, true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // If not FMS controlled add to teleop init too (for practice match and Red/Blue alliance need to be correctly set)
    checkDSUpdate();

    // RANDOM LED for autonomous
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> m_RunLEDPattern.setPattern(LedPatterns.RANDOMLED)));
    return m_autoManager.getAutonomousCommand();
    //TODO determine if autoManager needs to have andThen(() -> m_robotDrive.drive(0, 0, 0, false,false));
  }

  /**
   * Method to run before teleop starts, needed to help reset April Tag direction before teleop if operator does not do 
   * autonomous first.
   */
  public void teleopInit() {
    // Alliance color LED for for LED (but toggle cone/cube over take it)
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> m_RunLEDPattern.setPattern(LedPatterns.ALLIANCE)));
    // If not FMS controlled add to teleop init too (for practice match and Red/Blue alliance need to be correctly set)
    if(!DriverStation.isFMSAttached()) {
      m_robotDrive.setOriginBasedOnAlliance();
    }

}
  

  public void periodic() {
    // If not FMS controlled add to teleop init too (for practice match and Red/Blue alliance need to be correctly set)
    checkDSUpdate();

    m_fieldSim.periodic();
    m_mechanismSimulator.periodic();

    // TODO: Fix, This crashes code
    // if(m_timeOfFlight.sensorDetected()){
    //   m_CurrentPitchIntakeCommand = m_LowPitchIntakeCommand;
    // }
    // else{
    //   m_CurrentPitchIntakeCommand = m_HighPitchIntakeCommand;
    // }
  }

  public void disabledInit() {
    CommandScheduler.getInstance().schedule(m_RunLEDPattern);
    CommandScheduler.getInstance().schedule(new InstantCommand(() -> m_RunLEDPattern.setPattern(LedPatterns.DISABLED)));
  }

}