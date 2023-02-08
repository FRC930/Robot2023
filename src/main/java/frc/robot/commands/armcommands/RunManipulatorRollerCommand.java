package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class RunManipulatorRollerCommand extends CommandBase{

    private final double ROLLER_SPEED = 0.5;
    private final double VOLTAGE_LIMIT = 10;

    private ManipulatorSubsystem manipulator;

    /**
     * Runs the rollers on the manipulator until voltage spikes.
     * 
     * @param manipulatorSubsystem The manipulator subsystem
     */
    public RunManipulatorRollerCommand(ManipulatorSubsystem manipulatorSubsystem) {
        manipulator = manipulatorSubsystem;
    }

    @Override
    public void initialize() {
        manipulator.setRollerSpeed(ROLLER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return manipulator.getRollerVoltage() > VOLTAGE_LIMIT;
    }
}
