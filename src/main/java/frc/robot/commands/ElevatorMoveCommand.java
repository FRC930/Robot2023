package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorMoveCommand extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double desiredPosition;

    public ElevatorMoveCommand(ElevatorSubsystem elevatorSubsystem, double desiredPosition) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.desiredPosition = desiredPosition;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setTargetElevatorPosition(desiredPosition);
    }
    
    @Override
    public boolean isFinished(){
        return true;
    }
}
