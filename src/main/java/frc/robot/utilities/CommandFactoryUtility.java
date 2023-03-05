package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ElevatorMoveCommand;
import frc.robot.commands.ExtendIntakeCommand;
import frc.robot.commands.IntakeRollerCommand;
import frc.robot.commands.armcommands.RunManipulatorRollerCommand;
import frc.robot.commands.armcommands.SetArmDegreesCommand;
import frc.robot.subsystems.ExtendIntakeMotorSubsystem;
import frc.robot.subsystems.IntakeRollerMotorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class CommandFactoryUtility {

    private static double FACTOR = .68;
    private static final double ELEVATOR_INTAKE_HEIGHT = 12.0 * FACTOR; // 1.28/1.756  ;
    private static final double ARM_INTAKE_ANGLE = -33.0;
    private static final double MANIPULATOR_INTAKE = 25.0;

    private static final double ELEVATOR_UPRIGHT_INTAKE_HEIGHT = 17.4 * FACTOR; // 1.28/1.756  ;
    private static final double ARM_UPRIGHT_INTAKE_ANGLE = -22.0;
    private static final double MANIPULATOR_UPRIGHT_INTAKE = 4.5;

    private static final double ELEVATOR_BACK_INTAKE_HEIGHT = 14.0 * FACTOR; //not sure if correct?
    private static final double ARM_BACK_INTAKE_ANGLE = -184.5;
    private static final double MANIPULATOR_BACK_INTAKE = 260.0;

    private static final double ELEVATOR_HIGH_SCORE_HEIGHT =  52.0 * FACTOR; // 1.28/1.756  ;
    private static final double ARM_HIGH_SCORE_ANGLE = 35.0;
    private static final double MANIPULATOR_HIGH_SCORE = 1.0;

    private static final double ELEVATOR_MID_SCORE_HEIGHT =  22.0  * FACTOR; // 1.28/1.756  ;
    private static final double ARM_MID_SCORE_ANGLE = 35.0;
    private static final double MANIPULATOR_MID_SCORE = 0.0;

    private static final double ELEVATOR_LOW_SCORE_HEIGHT = 12.0 * FACTOR; // 1.28/1.756  ;
    private static final double ARM_LOW_SCORE_ANGLE = -25.0;
    private static final double MANIPULATOR_LOW_SCORE = 25.0;


    private CommandFactoryUtility() {}

    public static Command createScoreCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        double elevatorHeight,
        double waitSecondAfterElevator,
        double armPosition,
        double manipulatorPosition,
        double waitSecondArm) {
        Command command;
        
        command = new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(elevatorHeight))
            .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand()
                .withTimeout(waitSecondAfterElevator))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, manipulatorPosition))
            .andThen(m_armSubsystem.createWaitUntilAtAngleCommand()
                .withTimeout(waitSecondArm/2.0))
            .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand()
                .withTimeout(waitSecondArm/2.0))
            .andThen(new WaitCommand(0.3)) // TODO WHY waiting
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED));

        return command;
    }
    
    public static Command createScoreHighCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {        

        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_HIGH_SCORE_HEIGHT, 
            2.0, 
            ARM_HIGH_SCORE_ANGLE, 
            MANIPULATOR_HIGH_SCORE, 
            0.0);
    }

    public static Command createScoreMediumCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_MID_SCORE_HEIGHT, 
            1.0, 
            ARM_MID_SCORE_ANGLE, 
            MANIPULATOR_MID_SCORE, 
            1.0);
    }

    public static Command createScoreLowCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_LOW_SCORE_HEIGHT, 
            1.0, 
            ARM_LOW_SCORE_ANGLE, 
            MANIPULATOR_LOW_SCORE, 
            1.5);
    }

    public static Command createStowArmCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        final Command command = new ParallelCommandGroup(
            new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(0)),
            new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ArmSubsystem.STOW_POSITION, 
                ManipulatorSubsystem.STOW_POSITION))
            ); 

        return command;
    }


    private static Command createArmIntakeCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        double elevatorHeight,
        double waitSecond,
        double armPosition,
        double manipulatorPosition) {
        Command command;
        command = new SequentialCommandGroup(
            new RunManipulatorRollerCommand(m_manipulatorSubsystem,
                ManipulatorSubsystem.ROLLER_INTAKE_SPEED),
            new ElevatorMoveCommand(m_elevatorSubsystem, 
                Units.inchesToMeters(elevatorHeight)),
            m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(waitSecond),
            new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, 
                manipulatorPosition));

        return command;
    }

    public static Command createArmIntakeLowCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createArmIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_INTAKE_HEIGHT, 
            1.0, 
            ARM_INTAKE_ANGLE, 
            MANIPULATOR_INTAKE);
    }

    public static Command createArmIntakeUpRightCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,

        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createArmIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_UPRIGHT_INTAKE_HEIGHT, 
            1.0, 
            ARM_UPRIGHT_INTAKE_ANGLE, 
            MANIPULATOR_UPRIGHT_INTAKE);
    }

    public static Command createExtendIntakeCommand(
        ExtendIntakeMotorSubsystem extendIntakeMotorSubsystem, 
        IntakeRollerMotorSubsystem intakeRollerMotorSubsystem) {
        return new ExtendIntakeCommand(-6, 
            extendIntakeMotorSubsystem)
            .andThen(createIntakeRollerCommand(intakeRollerMotorSubsystem));
    }

    public static Command createIntakeRollerCommand(
        IntakeRollerMotorSubsystem m_IntakeRollerMotorSubsystem) {
        return new IntakeRollerCommand(2, 
        m_IntakeRollerMotorSubsystem);
    }

    public static Command createSingleSubstationCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        final Command command = 
            new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(0))
                .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(0.5))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ArmSubsystem.SUBSTATION_POSITION, 
                ManipulatorSubsystem.SUBSTATION_POSITION
                ))
                .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
                .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED)); //TODO constant

        return command;
    }
    //Create command for pitch intake


    /**
     * addAutoCommandEvent
     *  Add event command to event map for autonomous paths
     * 
     * @param eventCommandMap
     * @param eventName
     * @param m_elevatorSubsystem
     * @param m_armSubsystem
     * @param m_manipulatorSubsystem
     */
    public static void addAutoCommandEvent( Map<String, Command> eventCommandMap, 
                                            String eventName, 
                                            ElevatorSubsystem m_elevatorSubsystem, 
                                            ArmSubsystem m_armSubsystem, 
                                            ManipulatorSubsystem m_manipulatorSubsystem) {
        Command autoCommand = null;
        switch(eventName) {
            case "scoreHighCone":
                autoCommand = CommandFactoryUtility.createScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)
                                .andThen(new WaitCommand(0.5)) //pause after scoring
                                .andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));
                        
                break;
            case "scoreMidCone":
                autoCommand = CommandFactoryUtility.createScoreMediumCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)
                                .andThen(new WaitCommand(3)) //pause after scoring
                                .andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));
                break;
            case "armIntakeCone":
                autoCommand = CommandFactoryUtility.createArmIntakeLowCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
                                //.andThen(new WaitCommand(2))
                                //.andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)));
                break;
            case "armIntakeUprightCone":
                autoCommand = CommandFactoryUtility.createArmIntakeUpRightCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
                                //.andThen(new WaitCommand(2))
                                //.andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)));
                break;
            case "waitTilArmDown":
                autoCommand = m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5)
                .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5));   
                break;
            case "stowArm":
                autoCommand = new WaitCommand(0.5) // TODO why not check distance (low intake) versus a wait
                    // .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
                    // .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))   
                    .andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));
                break;
            case "scoreHighElevator":
                autoCommand = new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_HIGH_SCORE_HEIGHT));
                // No wait needed to spaced out in pathplanner 
                break;
            case "scoreHighArm":
                autoCommand = new SetArmDegreesCommand(m_armSubsystem,  ARM_HIGH_SCORE_ANGLE);
                // No wait needed to spaced out in pathplanner
                break;
            case "scoreHighManipulator":
                autoCommand = new SetArmDegreesCommand(m_manipulatorSubsystem, MANIPULATOR_HIGH_SCORE)
                .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand()
                .withTimeout(0.2))
                .andThen(m_armSubsystem.createWaitUntilAtAngleCommand()
                    .withTimeout(0.2))
                .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand()
                    .withTimeout(0.2))
                .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED));
                break;
            case "intakeElevatorPos":
                autoCommand =  new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED)
                .andThen(new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_INTAKE_HEIGHT)));
                // TODO why were we using waitUntil on intake commands
                break;
            case "intakeArmPos":
                autoCommand = new SetArmDegreesCommand(m_armSubsystem, ARM_INTAKE_ANGLE);
                // TODO why were we using waitUntil on intake commands
                break;
            case "intakeManipulatorPos":
                autoCommand = new SetArmDegreesCommand(m_manipulatorSubsystem,  MANIPULATOR_INTAKE);
                // TODO why were we using waitUntil on intake commands
                break;
            case "backIntakeElevatorPos":
                autoCommand =  new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED)
                .andThen(new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_BACK_INTAKE_HEIGHT)));
                // TODO why were we using waitUntil on intake commands
                break;
            case "backIntakeArmPos":
                autoCommand = new SetArmDegreesCommand(m_armSubsystem, ARM_BACK_INTAKE_ANGLE);
                // TODO why were we using waitUntil on intake commands
                break;
            case "backIntakeManipulatorPos":
                autoCommand = new SetArmDegreesCommand(m_manipulatorSubsystem,  MANIPULATOR_BACK_INTAKE);
                // TODO why were we using waitUntil on intake commands
                break;
            case "scoreHighNoStow":
                autoCommand = CommandFactoryUtility.createScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)
                .andThen(new WaitCommand(0.3));
                break;
        }

        if(autoCommand != null) {
            eventCommandMap.put(eventName, autoCommand);
        } else {
            DriverStation.reportWarning("Unable to add event name"+eventName+" given not declared!", 
                Thread.currentThread().getStackTrace());
        }
     
    }


}