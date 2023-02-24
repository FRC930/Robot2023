package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorMoveCommand extends CommandBase {
    private final ElevatorSubsystem m_elevatorSubsystem;
    private final double m_desiredPosition;

    /**
     * <h3>ElevatorMoveCommand</h3>
     * 
     * Command to move elevator to a certain position.
     * @param elevatorSubsystem - The elevator subsystem
     * @param position - The desired position in meters
     */
    public ElevatorMoveCommand(ElevatorSubsystem elevatorSubsystem, double position) {
        m_elevatorSubsystem = elevatorSubsystem;
        m_desiredPosition = position;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        m_elevatorSubsystem.setTargetElevatorPosition(m_desiredPosition);
    }
    
    @Override
    public boolean isFinished(){
        return true;
    }
}
