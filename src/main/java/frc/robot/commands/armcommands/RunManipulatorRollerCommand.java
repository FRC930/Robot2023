package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class RunManipulatorRollerCommand extends CommandBase{

    private double ROLLER_SPEED = 0.1;

    private ManipulatorSubsystem manipulator;

    /**
     * Runs the rollers on the manipulator until voltage spikes.
     * 
     * @param manipulatorSubsystem The manipulator subsystem
     */
    public RunManipulatorRollerCommand(ManipulatorSubsystem manipulatorSubsystem, double ROLLER_SPEED) {
        manipulator = manipulatorSubsystem;
        this.ROLLER_SPEED = ROLLER_SPEED;
    }

    @Override
    public void initialize() {
        manipulator.setRollerSpeed(ROLLER_SPEED);
    }
}
