package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveDrive;

// TODO why keeping (swervedrive)
public class RotationMathUtility {
    private SwerveDrive m_swerveDrive;
    private double m_turningSpeed;
    private double m_turningAngle;
    private double m_angleOffset;
    
    /**
     * Determines a desired spot on the field based on a given coordinate.
     * 
     * @param currentPose2D - Current location on the field
     * @param targetPose2d - Desired location on the field
     * @param robotHeading - Current heading of the robot
     * @return Speed at which to rotate
     */
    public double rotationSpeed(Pose2d currentPose2D, Pose2d targetPose2d, double robotHeading){
        m_turningSpeed = 0.0;
        m_angleOffset = currentPose2D.getRotation().getRadians();
        //Finds the turning speed
        double cx = currentPose2D.getX();
        double cy = currentPose2D.getY();
        double x = targetPose2d.getX();
        double y = targetPose2d.getY();

        //Calculates the angle using atan2 and adjusting using the robots current position
        double calculatedheading = Math.atan2(y - cy, x - cx);
        m_turningAngle = (calculatedheading - m_angleOffset);

        Logger.getInstance().recordOutput("RotateCommand/Angle1", m_turningAngle);
        //If turningAngle wants to turn to the right more than 180 degrees, it will turn that distance to the left
        if (m_turningAngle > Math.PI) {
            m_turningAngle = -1.0 * (Math.PI - (m_turningAngle - Math.PI));
        }
        Logger.getInstance().recordOutput("RotateCommand/Angle2", m_turningAngle);
        //Finds the turning speed
        m_turningSpeed = -1.0 * MathUtil.clamp((m_swerveDrive.getAutoThetaController().calculate(m_turningAngle, 0)), -1, 1);

        return m_turningSpeed;
    }
    
}
