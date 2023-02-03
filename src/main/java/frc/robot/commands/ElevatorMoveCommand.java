package frc.robot.commands;

import javax.management.loading.PrivateClassLoader;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double desiredPosition;

    // The min hight of the elevator
    private final int MinElevatorHight = 5;
    // The max hight of the elevator
    private final int MaxElevatorHight = 27;

    public ElevatorMoveCommand(ElevatorSubsystem elevator, double position) {
        elevatorSubsystem = elevator;
        desiredPosition = position;
        addRequirements(elevatorSubsystem);
    }
    // TODO: Find better alowed error

    @Override
    public void execute() {
        /*
        if (elevatorSubsystem.getElevatorPosition() > desiredPosition - 5 && desiredPosition >= MinElevatorHight) {
            elevatorSubsystem.setElevatorSpeed(-0.5);
        } else if (elevatorSubsystem.getElevatorPosition() < desiredPosition + 5 && desiredPosition <= MaxElevatorHight) {
            elevatorSubsystem.setElevatorSpeed(0.5);
        } else {
            elevatorSubsystem.stopMotors();
        }
        */
    }
}
