package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase{

    private static double allowedError = 2.5;

    private double speed;
    private ArmSubsystem arm;
    private double elbowPos;
    private double shoulderPos;

    public MoveArmCommand(ArmSubsystem armSubsystem, double speed, double elbowPosition, double shoulderPosition) {
        this.speed = speed;
        arm = armSubsystem;
        elbowPos = elbowPosition;
        shoulderPos = shoulderPosition;
    }

    @Override
    public void execute() {
        if (arm.getElbowPosition() > elbowPos - allowedError) {
            arm.setElbowSpeed(-speed);
        } else if (arm.getElbowPosition() < elbowPos + allowedError) {
            arm.setElbowSpeed(speed);
        } else {
            arm.stopElbowMotor();
        }

        if (arm.getShoulderPosition() > shoulderPos - allowedError) {
            arm.setShoulderSpeed(-speed);
        } else if (arm.getShoulderPosition() < shoulderPos + allowedError) {
            arm.setShoulderSpeed(speed);
        } else {
            arm.stopShoulderMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return (arm.getElbowPosition() < elbowPos - allowedError
                && arm.getElbowPosition() > elbowPos + allowedError
                && arm.getShoulderPosition() < shoulderPos - allowedError
                && arm.getShoulderPosition() > shoulderPos + allowedError);
    }
}
