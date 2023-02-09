package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveDrive;


public class RotationMathUtility {
    private SwerveDrive m_swerveDrive;
    private double turningSpeed;
    private double turningAngle;
    private double m_angleOffset;
    
    public double rotationSpeed(Pose2d currentPose2D, Pose2d m_targetPose2d, double robotHeading){
        turningSpeed = 0.0;
        m_angleOffset = currentPose2D.getRotation().getRadians();
        //Finds the turning speed
        double cx = currentPose2D.getX();
        double cy = currentPose2D.getY();
        double x = m_targetPose2d.getX();
        double y = m_targetPose2d.getY();

        //Calculates the angle using atan2 and adjusting using the robots current position
        double calculatedheading = Math.atan2(y - cy, x - cx);
        turningAngle = (calculatedheading - m_angleOffset);

        Logger.getInstance().recordOutput("RotateCommand/Angle1", turningAngle);
        //If turningAngle wants to turn to the right more than 180 degrees, it will turn that distance to the left
        if (turningAngle > Math.PI) {
            turningAngle = -1.0 * (Math.PI - (turningAngle - Math.PI));
        }
        Logger.getInstance().recordOutput("RotateCommand/Angle2", turningAngle);
        //Finds the turning speed
        turningSpeed = -1.0 * MathUtil.clamp((m_swerveDrive.getAutoThetaController().calculate(turningAngle, 0)), -1, 1);

        return turningSpeed;
    }
    
}
