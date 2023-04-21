package frc.robot.utilities;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private final static double MANIPULATOR_REDUCTION = 0.0;
    private final static double INTAKE_EXTEND_VOLTAGE = 6.0;
    private final static double INTAKE_ROLLER_VOLTAGE = 5.0;
    // Conversion factor for Elevator (pulley sizes changed)
    private static double FACTOR = .68;

    //Arm Intake ground/low
    public static final double ELEVATOR_INTAKE_HEIGHT = 12.0 * FACTOR; // 1.28/1.756  ;
    public static final double ARM_INTAKE_ANGLE = -24.0;
    public static final double MANIPULATOR_INTAKE = 17.0 + MANIPULATOR_REDUCTION - 4;

    // Extend Ground Intake
    // TODO: Check These Angles
    public static final double ELEVATOR_GROUNDINTAKE_HEIGHT = 0.0;
    public static final double ARM_GROUNDINTAKE_ANGLE = 30.0; //changed wednesday night 4/12/23 from -20 to -22 need to test 
    public static final double MANIPULATOR_GROUNDINTAKE = -70.0;


    // Arm Intake UpRight cone
    public static final double ELEVATOR_UPRIGHT_INTAKE_HEIGHT = 17.4 * FACTOR; // 1.28/1.756  ;
    public static final double ARM_UPRIGHT_INTAKE_ANGLE = -15.5;
    public static final double MANIPULATOR_UPRIGHT_INTAKE = 8.5 + MANIPULATOR_REDUCTION;

    //Arm substation
    public static final double ELEVATOR_SUBSTATION_HEIGHT = 26.2; //29.1; //26.0 * FACTOR; //not sure if correct?
    public static final double ARM_SUBSTATION_ANGLE = 207.7; //209.4; //200.0;
    public static final double MANIPULATOR_SUBSTATION = 147.7 + MANIPULATOR_REDUCTION; //155.1; //155.0;

    //Arm Double substation
    public static final double ELEVATOR_DOUBLE_SUBSTATION_HEIGHT = 0.0; //12.2; //14.4 Maybe use these values
    public static final double ARM_DOUBLE_SUBSTATION_ANGLE = 75; //86.5; //80.2
    public static final double MANIPULATOR_DOUBLE_SUBSTATION = 45; // 14.6 + MANIPULATOR_REDUCTION; //-5.8 //2.8;

    // Arm Back intake DONT USE
    // TODO DONT USE YET WRIST WILL CRASH INTO ARM (need to find way to move safely)
    public static final double ELEVATOR_BACK_INTAKE_HEIGHT = 13.6;  // NO CONVESION FACTOR
    public static final double ARM_BACK_INTAKE_ANGLE = 197.0;
    public static final double MANIPULATOR_BACK_INTAKE = 244.0 + MANIPULATOR_REDUCTION;

    //Values for backwards cone pickup in autonomous
    // public static final double AUTO_ELEVATOR_BACK_INTAKE_HEIGHT = 13;  // NO CONVESION FACTOR
    // public static final double AUTO_ARM_BACK_INTAKE_ANGLE = 205.0;
    // public static final double AUTO_MANIPULATOR_BACK_INTAKE = 250.0;

    public static final double ELEVATOR_BACK_CUBE_INTAKE_HEIGHT = 11.3;  // NO CONVESION FACTOR
    public static final double ARM_BACK_CUBE_INTAKE_ANGLE = 201.0;
    public static final double MANIPULATOR_BACK_CUBE_INTAKE = 250.0 + MANIPULATOR_REDUCTION;

    // High Score
    public static final double ELEVATOR_HIGH_SCORE_HEIGHT =  50.0 * FACTOR; // 1.28/1.756  ;
    public static final double ARM_HIGH_SCORE_ANGLE = 52.0; 
    public static final double MANIPULATOR_HIGH_SCORE = -3.0 + MANIPULATOR_REDUCTION;

    // Mid Score
    public static final double ELEVATOR_MID_SCORE_HEIGHT =  22.0  * FACTOR; // 1.28/1.756  ;
    public static final double ARM_MID_SCORE_ANGLE = 58.0; 
    public static final double MANIPULATOR_MID_SCORE = -6.0 + MANIPULATOR_REDUCTION;

    // Low Score
    public static final double ELEVATOR_LOW_SCORE_HEIGHT = 0.0 * FACTOR; // 1.28/1.756  ;
    public static final double ARM_LOW_SCORE_ANGLE = 70.0;
    public static final double MANIPULATOR_LOW_SCORE = -25.0 + MANIPULATOR_REDUCTION;



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
            // .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand()
            //     .withTimeout(waitSecondAfterElevator))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, manipulatorPosition));
        // iF wish to wait for arm/manipulator gets to position than release or DONT release DRIVER will control this
        if(waitSecondArm >= 0.0) {
            command = command
            .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand() // needed because violent fast arm movement for scoring during auto
                .withTimeout(waitSecondAfterElevator))
            .andThen(m_armSubsystem.createWaitUntilAtAngleCommand()
                .withTimeout(waitSecondArm/2.0))
            .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand()
                .withTimeout(waitSecondArm/2.0))
            // .andThen(new WaitCommand(0.1)) // TODO WHY waiting
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED)
            );
        }

        return command;
    }
    
    public static Command createScoreHighCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        boolean releaseAtEnd) {        

        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_HIGH_SCORE_HEIGHT, 
            0.5, 
            ARM_HIGH_SCORE_ANGLE, 
            MANIPULATOR_HIGH_SCORE, 
            releaseAtEnd?0.4:-1.0);
    }

    public static Command createAutoScoreHighCommand(ElevatorSubsystem m_elevatorSubsystem, ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        return new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
            .andThen(CommandFactoryUtility.createScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, true)
            .andThen(new WaitCommand(0.18)) //pause after scoring
            .andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)));
    }

    public static Command createAutoScoreMidCommand(ElevatorSubsystem m_elevatorSubsystem, ArmSubsystem m_armSubsystem,
    ManipulatorSubsystem m_manipulatorSubsystem) {
    return new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
        .andThen(CommandFactoryUtility.createScoreMediumCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, true)
        .andThen(new WaitCommand(0.18)) //pause after scoring
        .andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)));
}

    public static Command createScoreMediumCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        boolean releaseAtEnd) {
        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_MID_SCORE_HEIGHT, 
            1.0, 
            ARM_MID_SCORE_ANGLE, 
            MANIPULATOR_MID_SCORE, 
            releaseAtEnd?1.0:-1.0);
    }

    public static Command createScoreLowCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem,
        boolean releaseAtEnd) {
        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_LOW_SCORE_HEIGHT, 
            1.0, 
            ARM_LOW_SCORE_ANGLE, 
            MANIPULATOR_LOW_SCORE, 
            releaseAtEnd?1.5:-1.0);
    }

    public static Command createStowArmCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        final Command command = 
            new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED)
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ArmSubsystem.STOW_POSITION, 
                ManipulatorSubsystem.STOW_POSITION))
            .andThen(m_armSubsystem.createWaitUntilLessThanAngleCommand(170.0))    
            .andThen(m_armSubsystem.createWaitUntilGreaterThanAngleCommand(45.0))    
            .andThen(new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(0))) ;
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
            // m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(waitSecond),
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

    public static Command createArmBackIntakeCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createArmIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_BACK_INTAKE_HEIGHT, 
            1.0, 
            ARM_BACK_INTAKE_ANGLE, 
            MANIPULATOR_BACK_INTAKE);
    }

    public static Command createArmBackCubeIntakeCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createArmIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            ELEVATOR_BACK_CUBE_INTAKE_HEIGHT, 
            1.0, 
            ARM_BACK_CUBE_INTAKE_ANGLE, 
            MANIPULATOR_BACK_CUBE_INTAKE);
    }

    /**
   * <h3>createExtendIntakeCommand<h3/>
   * 
   * creates the enxtendIntakeCommand and determines whether or not to use the rollers for the ground intake
   * 
   * @param ExtendIntakeMotorSubsystem extendIntakeMotorSubsystem subsystem for extending the intake motors
   * @param IntakeRollerMotorSubsystem intakeRollerMotorSubsystem subsystem for the intake roller motors
   * @param boolean runRoller determines whether or not to set roller speed or not
   * 
   */
    public static Command createExtendIntakeCommand(
        ExtendIntakeMotorSubsystem extendIntakeMotorSubsystem, 
        IntakeRollerMotorSubsystem intakeRollerMotorSubsystem, boolean runRoller) {
            double rollerSpeed = 0.0;
            if(runRoller ){
                rollerSpeed = -INTAKE_ROLLER_VOLTAGE;
            }
        return 
            new ExtendIntakeCommand(-INTAKE_EXTEND_VOLTAGE, extendIntakeMotorSubsystem)
                .withTimeout(0.5)
            .andThen(new IntakeRollerCommand(rollerSpeed, intakeRollerMotorSubsystem)); 
    }

    public static Command createRetractIntakeCommand(
        ExtendIntakeMotorSubsystem extendIntakeMotorSubsystem, 
        IntakeRollerMotorSubsystem intakeRollerMotorSubsystem) {
        return new ExtendIntakeCommand(INTAKE_EXTEND_VOLTAGE, extendIntakeMotorSubsystem)
            .withTimeout(0.5)
        .andThen(new IntakeRollerCommand(0.0, intakeRollerMotorSubsystem));
    }

    public static Command createSingleSubstationCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        final Command command = 
            new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_SUBSTATION_HEIGHT))
                .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(0.5))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ARM_SUBSTATION_ANGLE, 
                MANIPULATOR_SUBSTATION
                ))
                .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
                .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED));

        return command;
    }

    public static Command createDoubleSubstationCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        final Command command = 
            new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_DOUBLE_SUBSTATION_HEIGHT))
                .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(0.5))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ARM_DOUBLE_SUBSTATION_ANGLE, 
                MANIPULATOR_DOUBLE_SUBSTATION
                ))
                .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
                .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.DOUBLE_SUBSTATION_ROLLER_INTAKE_SPEED));

        return command;
    }

    public static Command createScoreGroundCubeCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        
        final Command command = 
            new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_INTAKE_HEIGHT))
                .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(0.5))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem,
                ARM_INTAKE_ANGLE, MANIPULATOR_INTAKE))
                .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5)
                .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5)))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED));

        return command;
    }
    /**<h3>createGroundIntakeExtendCommand</h3>
     * Extends the ground intake and positions the arm in order to pick up a game peice
     * 
     * @param extendIntakeSubsystem
     * @param intakeSubsystem
     * @param m_elevatorSubsystem
     * @param m_armSubsystem
     * @param m_manipulatorSubsystem
     * @return
     */
    public static Command createGroundIntakeExtendCommand(
        ExtendIntakeMotorSubsystem extendIntakeSubsystem,
        IntakeRollerMotorSubsystem intakeSubsystem,
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        
        final Command command = 
            new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED)
            .andThen(createExtendIntakeCommand(extendIntakeSubsystem, intakeSubsystem, true))
            .andThen(new WaitCommand(0.05)) // pause before we intake the peice
            .andThen(
                new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_GROUNDINTAKE_HEIGHT))
                )
            .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand().withTimeout(0.5))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ARM_GROUNDINTAKE_ANGLE, 
                MANIPULATOR_GROUNDINTAKE
                ))
            .andThen(m_armSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
            .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand().withTimeout(0.5))
         .andThen(new WaitCommand(0.3)) 

        .andThen(m_manipulatorSubsystem.waitUntilCurrentPast(10.0))
        .andThen(new IntakeRollerCommand(0.0, intakeSubsystem))
        .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED))
        .andThen(createGroundIntakeRetractCommand(extendIntakeSubsystem, intakeSubsystem, m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem))
        .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED));
        // .andThen(new WaitCommand(0.75))
        // .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED));


        return command;
    }


    /**<h3>createGroundIntakeRetractCommand</h3>
     * 
     * Retracts the ground intake and puts the arm in the stow position
     * 
     * @param extendIntakeSubsystem
     * @param intakeSubsystem
     * @param m_elevatorSubsystem
     * @param m_armSubsystem
     * @param m_manipulatorSubsystem
     * @return
     */
    public static Command createGroundIntakeRetractCommand(
        ExtendIntakeMotorSubsystem extendIntakeSubsystem,
        IntakeRollerMotorSubsystem intakeSubsystem,
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        
        final Command command = 
            createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)
            //.andThen(new WaitCommand(0.1))
            .andThen(createRetractIntakeCommand(extendIntakeSubsystem, intakeSubsystem));
            

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
                autoCommand = CommandFactoryUtility.createScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem,
                                    true)
                                .andThen(new WaitCommand(0.18)) //pause after scoring
                                .andThen(CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem));
                        
                break;
            case "scoreMidCube":
                autoCommand = //CommandFactoryUtility.createStowArmCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem)
                    new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.SHOOT_SPEED)
                    .andThen(new WaitCommand(0.4)); //pause after scoring
                break;
                
            case "scoreMidCone":
                autoCommand = CommandFactoryUtility.createScoreMediumCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem,
                                    true)
                                .andThen(new WaitCommand(0.18)) //pause after scoring
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
            case "scoreHighManipulatorAndNotRelease":
                autoCommand = new SetArmDegreesCommand(m_manipulatorSubsystem, MANIPULATOR_HIGH_SCORE);
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
            // TODO DONT USE YET (wrist issues)
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
            case "backCubeIntakeElevatorPos":
                autoCommand =  new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED)
                .andThen(new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(ELEVATOR_BACK_CUBE_INTAKE_HEIGHT)));
                // TODO why were we using waitUntil on intake commands
                break;
            case "backCubeIntakeArmPos":
                autoCommand = new SetArmDegreesCommand(m_armSubsystem, ARM_BACK_CUBE_INTAKE_ANGLE);
                // TODO why were we using waitUntil on intake commands
                break;
            case "backCubeIntakeManipulatorPos":
                autoCommand = new SetArmDegreesCommand(m_manipulatorSubsystem,  MANIPULATOR_BACK_CUBE_INTAKE);
                // TODO why were we using waitUntil on intake commands
                break;
            case "scoreHighNoStow":
                autoCommand = CommandFactoryUtility.createScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
                    true)
                .andThen(new WaitCommand(0.35));
                break;
            case "manipulatorHold":
                autoCommand =  new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.HOLD_SPEED);
                // TODO why were we using waitUntil on intake commands
                break;
            case "scoreGroundCube":
                autoCommand = CommandFactoryUtility.createScoreGroundCubeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
                break;
            case "backCubeIntake":
                autoCommand = CommandFactoryUtility.createArmBackCubeIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem);
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