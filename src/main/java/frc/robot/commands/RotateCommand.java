package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;

/**
 * <h3>RotateCommand</h3>
 * 
 * Turns towards a desired spot on the field based on a given coordinate.
 * 
 */
public class RotateCommand extends CommandBase{

    private final double Aim_Deadband = 0.2;

    private SwerveDrive m_swerveDrive;

    private Pose2d m_targetPose2d;
    private Pose2d currentPose2d;

    private double throttle;
    private double strafe;
    private double turningAngle;
    private boolean isFieldRelative;
    private boolean isOpenLoop;

    private double m_angleOffset;

    /**
     * <h3>RotateCommand</h3>
     * 
     * Turns towards a desired spot on the field based on a given coordinate.
     * 
     * @param targetPose a desired spot on the field
     * @param swerveDrive Drive subsystem
     */
    public RotateCommand(Pose2d targetPose, SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
        m_targetPose2d = targetPose;
        m_angleOffset = 0.0;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        //Gets current position and heading of the robot and target position.
        currentPose2d = m_swerveDrive.getPose();
        m_angleOffset = currentPose2d.getRotation().getRadians();
        double cx = currentPose2d.getX();
        double cy = currentPose2d.getY();
        double x = m_targetPose2d.getX();
        double y = m_targetPose2d.getY();

        //Calculates the angle using atan2 and adjusting using the robots current position
        turningAngle = -(Math.atan2(y - cy, x - cx) - m_angleOffset);

        //If turningAngle wants to turn to the right more than 180 degrees, it will turn that distance to the left
        if (turningAngle > Math.PI) {
            turningAngle = -Math.PI + (turningAngle - Math.PI);
        }

        //Logs information regarding the command
        Logger.getInstance().recordOutput("RotateCommand/Angle", turningAngle);
        Logger.getInstance().recordOutput("RotateCommand/cx", cx);
        Logger.getInstance().recordOutput("RotateCommand/cy", cy);
        Logger.getInstance().recordOutput("RotateCommand/x", x);
        Logger.getInstance().recordOutput("RotateCommand/y", y);
        Logger.getInstance().recordOutput("RotateCommand/offset", m_angleOffset);
        
        //Turns the robot
        m_swerveDrive.drive(throttle, strafe, turningAngle, isFieldRelative, isOpenLoop);
    }

    @Override
    public boolean isFinished() {
        //Deadband for the robot aiming in radians
        //TODO adjust deadband to something more realistic
        if (m_angleOffset > turningAngle + Aim_Deadband && m_angleOffset < turningAngle - Aim_Deadband) {
            return true;
        } else {
            return false;
        }
        
    }

    @Override
    public void end(boolean interrupted){

    }
}
