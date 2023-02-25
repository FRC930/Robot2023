package frc.robot.utilities;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
        //TODO remove old method
        // command = new SequentialCommandGroup(
        //     new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(elevatorHeight)),
        //     new WaitCommand(waitSecondAfterElevator),
        //     new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, manipulatorPosition),
        //     new WaitCommand(waitSecondArm),
        //     new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED)
        // );

        // command = new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(elevatorHeight))
        //     .andThen(new WaitCommand(waitSecondAfterElevator))
        //     .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, manipulatorPosition))
        //     .andThen(new WaitCommand(waitSecondArm))
        //     .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED));
            
        command = new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(elevatorHeight))
            .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand()
                .withTimeout(waitSecondAfterElevator))
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, manipulatorPosition))
            .andThen(m_armSubsystem.createWaitUntilAtAngleCommand()
                .withTimeout(waitSecondArm/2.0))
            .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand()
                .withTimeout(waitSecondArm/2.0))
            .andThen(new WaitCommand(0.5))
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.RELEASE_SPEED));

        return command;
    }
    
    public static Command createScoreHighCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {        

        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            55.0, 
            2.0, 
            35.0, 
            1.0, 
            0.0);
    }

    public static Command createScoreMediumCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            22.0, 
            1.0, 
            35.0, 
            0.0, 
            1.0);
    }

    public static Command createScoreLowCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        return createScoreCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            12.0, 
            1.0, 
            -25.0, 
            25.0, 
            1.5);
    }

    public static Command createStowArmCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {
        final Command command = new ParallelCommandGroup(
            new ElevatorMoveCommand(m_elevatorSubsystem, Units.inchesToMeters(0)),
            new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ArmSubsystem.STOW_POSITION, 
                ManipulatorSubsystem.STOW_POSITION
                ),
            new RunManipulatorRollerCommand(m_manipulatorSubsystem, 0.15)); //TODO constant

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
            new WaitCommand(waitSecond),
            new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, armPosition, 
                manipulatorPosition));

        return command;
    }

    public static Command createArmIntakeLowCommand(
        ElevatorSubsystem m_elevatorSubsystem,
        ArmSubsystem m_armSubsystem,
        ManipulatorSubsystem m_manipulatorSubsystem) {

        return createArmIntakeCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem, 
            12.0, 
            1.0, 
            -33.0, 
            25.0);
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
                .andThen(m_elevatorSubsystem.createWaitUntilAtHeightCommand()).withTimeout(0.5)
            .andThen(new SetArmDegreesCommand(m_armSubsystem, m_manipulatorSubsystem, 
                ArmSubsystem.SUBSTATION_POSITION, 
                ManipulatorSubsystem.SUBSTATION_POSITION
                ))
                .andThen(m_armSubsystem.createWaitUntilAtAngleCommand()).withTimeout(0.5)
                .andThen(m_manipulatorSubsystem.createWaitUntilAtAngleCommand()).withTimeout(0.5)
            .andThen(new RunManipulatorRollerCommand(m_manipulatorSubsystem, ManipulatorSubsystem.ROLLER_INTAKE_SPEED)); //TODO constant

        return command;
    }
    //Create command for pitch intake
}
