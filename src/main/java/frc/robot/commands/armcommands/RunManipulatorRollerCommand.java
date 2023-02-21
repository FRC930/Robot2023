package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class RunManipulatorRollerCommand extends CommandBase{

    private double m_rollerSpeed;

    private ManipulatorSubsystem m_manipulator;

    /**
     * <h3>RunManipulatorRollerCommand</h3>
     * 
     * Runs the rollers on the manipulator until voltage spikes.
     * @param manipulatorSubsystem The manipulator subsystem
     * @param speed Desired speed of roller
     */
    public RunManipulatorRollerCommand(ManipulatorSubsystem manipulatorSubsystem, double speed) {
        m_manipulator = manipulatorSubsystem;
        m_rollerSpeed = speed;
        addRequirements(manipulatorSubsystem);
    }

    @Override
    public void initialize() {
       m_manipulator.setRollerSpeed(m_rollerSpeed);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    
}
