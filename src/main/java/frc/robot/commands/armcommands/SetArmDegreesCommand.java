package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmDegreesCommand extends CommandBase{

    private ArmSubsystem arm;
    private double wristPos;
    private double shoulderPos;

    /**
     * Sets the positions of the shoulder and wrist motors to the desired positions.
     * 
     * @param armSubsystem The arm subsystem
     * @param wristPosition The desired wrist position in degrees
     * @param shoulderPosition The desired shoulder position in degrees
     */
    public SetArmDegreesCommand(ArmSubsystem armSubsystem, double wristPosition, double shoulderPosition) {
        arm = armSubsystem;
        wristPos = wristPosition;
        shoulderPos = shoulderPosition;
    }

    @Override
    public void execute() {
        arm.setWristPosition(wristPos);
        arm.setShoulderPosition(shoulderPos);
    }

    @Override
    public boolean isFinished() {
        return (arm.getShoulderPosition() == shoulderPos
                && arm.getWristPosition() == wristPos);
    }
}
