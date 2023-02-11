package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class SetArmDegreesCommand extends CommandBase{

    private ArmSubsystem arm;
    private ManipulatorSubsystem manipulator;
    private double manipulatorPos;
    private double armPos;

    /**
     * Sets the positions of the shoulder and wrist motors to the desired positions.
     * 
     * @param armSubsystem The arm subsystem
     * @param wristPosition The desired wrist position in degrees
     * @param shoulderPosition The desired shoulder position in degrees
     */
    public SetArmDegreesCommand(ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem, double armPosition, double manipulatorPosition) {
        arm = armSubsystem;
        manipulator = manipulatorSubsystem;
        manipulatorPos = manipulatorPosition;
        armPos = armPosition;
    }

    @Override
    public void execute() {
        manipulator.setPosition(manipulatorPos);
        arm.setPosition(armPos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
