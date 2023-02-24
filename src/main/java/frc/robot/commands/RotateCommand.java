package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import org.littletonrobotics.junction.Logger;

/**
 * <h3>RotateCommand</h3>
 * 
 * Turns towards a desired spot on the field based on a given coordinate
 */
public class RotateCommand extends CommandBase {

    private final double SPEED_REDUCTION = 1.0;

    private SwerveDrive m_swerveDrive;
    private double m_throttle;
    private double m_strafe;
    private double m_turningAngle;
    private boolean m_isFieldRelative;
    private boolean m_isOpenLoop;

    private Pose2d m_currentPose2d;
    private Pose2d m_targetPose2d;

    private double m_angleOffset;
    private double m_turningSpeed;
    private double m_calculatedHeading;

    private double m_cx;
    private double m_cy;
    private double m_x;
    private double m_y;

    private PIDController m_thetaController;

    /**
     * <h3>RotateCommand</h3>
     * 
     * Turns towards a desired spot on the field based on a given coordinate
     * 
     * @param targetPose  a desired spot on the field
     * @param swerveDrive Drive subsystem
     */
    public RotateCommand(Pose2d targetPose, SwerveDrive swerveDrive) {
        m_targetPose2d = targetPose;
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);

        m_throttle = 0.0;
        m_strafe = 0.0;
        m_turningSpeed = 0.0;
        m_isFieldRelative = true;
        m_isOpenLoop = true;

        m_angleOffset = 0.0;
        m_turningAngle = 0.0;
        m_calculatedHeading = 0.0;

        // TODO: determine PID values
        m_thetaController = new PIDController(0.0, 0.0, 0.0);

    }

    @Override
    public void execute() {
        // Gets current position and heading of the robot
        m_currentPose2d = m_swerveDrive.getPose();
        m_angleOffset = m_currentPose2d.getRotation().getRadians();

        // Gets the X and Y of the current position and target position
        m_cx = m_currentPose2d.getX();
        m_cy = m_currentPose2d.getY();
        m_x = m_targetPose2d.getX();
        m_y = m_targetPose2d.getY();

        // Calculates the angle using atan2 and adjusting using the robots current position
        m_calculatedHeading = Math.atan2(m_y - m_cy, m_x - m_cx);
        m_turningAngle = (m_calculatedHeading - m_angleOffset);
        Logger.getInstance().recordOutput("RotateCommand/Angle1", m_turningAngle);

        // If turningAngle wants to turn to the right more than 180 degrees, it will turn that distance to the left
        if (m_turningAngle > Math.PI) {
            m_turningAngle = -1.0 * (Math.PI - (m_turningAngle - Math.PI));
        }

        Logger.getInstance().recordOutput("RotateCommand/Angle2", m_turningAngle);

        // Finds the turning speed
        m_turningSpeed = -1.0 * MathUtil.clamp((m_thetaController.calculate(m_turningAngle, 0.0)), -1.0, 1.0);

        // Logs information regarding the command
        Logger.getInstance().recordOutput("RotateCommand/cx", m_cx);
        Logger.getInstance().recordOutput("RotateCommand/cy", m_cy);
        Logger.getInstance().recordOutput("RotateCommand/x", m_x);
        Logger.getInstance().recordOutput("RotateCommand/y", m_y);
        Logger.getInstance().recordOutput("RotateCommand/Offset", m_angleOffset);
        Logger.getInstance().recordOutput("RotateCommand/calculatedheading", m_calculatedHeading);
        Logger.getInstance().recordOutput("RotateCommand/TurningSpeed", m_turningSpeed);

        // Turns the robot
        m_swerveDrive.drive(m_throttle, m_strafe, m_turningSpeed * SPEED_REDUCTION, m_isFieldRelative, m_isOpenLoop);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
