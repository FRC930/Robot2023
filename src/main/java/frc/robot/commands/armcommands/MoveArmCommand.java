package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCommand extends CommandBase{

    private static double allowedError = 2.5;

    private double speed;
    private ArmSubsystem arm;
    private double wristPos;
    private double armPos;

    public MoveArmCommand(ArmSubsystem armSubsystem, double speed, double wristPosition, double armPosition) {
        this.speed = speed;
        arm = armSubsystem;
        wristPos = wristPosition;
        armPos = armPosition;
    }

    @Override
    public void execute() {
        if (arm.getWristPosition() > wristPos - allowedError) {
            arm.setWristSpeed(-speed);
        } else if (arm.getWristPosition() < wristPos + allowedError) {
            arm.setWristSpeed(speed);
        } else {
            arm.stopWristMotor();
        }

        if (arm.getArmPosition() > armPos - allowedError) {
            arm.setArmSpeed(-speed);
        } else if (arm.getArmPosition() < armPos + allowedError) {
            arm.setArmSpeed(speed);
        } else {
            arm.stopArmMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return (arm.getWristPosition() < wristPos - allowedError
                && arm.getWristPosition() > wristPos + allowedError
                && arm.getArmPosition() < armPos - allowedError
                && arm.getArmPosition() > armPos + allowedError);
    }
}
