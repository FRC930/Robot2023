package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.RotatePositions;
import frc.robot.utilities.RotationMathUtility;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * <h3>TeleopSwerve</h3>
 * 
 * Allows the driver to move the robot manually through controller input
 */
public class TeleopSwerve extends CommandBase {

    public final static double SLOW_SPEED = 0.4;
    public final static double NORMAL_SPEED = 1.0;
    private final double STICK_DEAD_BAND = 0.1;


    public double m_percentSpeed;

    private SwerveDrive m_Swerve;
    private RotatePositions m_rotatePositions;
    private CommandXboxController m_controller;
    private RotationMathUtility m_rotationMathUtility;
    private int m_translationAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;
    private boolean m_fieldRelative;
    private boolean m_openLoop;

    /**
     * <h3>TeleopSwerve</h3>
     * 
     * Allows the driver to move the robot manually through controller input
     * 
     * @param m_Swerve        The swerve drive that moves the robot
     * @param controller      The driver's controller
     * @param translationAxis Controller translation input
     * @param strafeAxis      Controller strafe input
     * @param rotationAxis    Controller rotation input
     * @param fieldRelative   Controlls relative to orientation
     * @param openLoop        Open Loop does not use PID values to correct inputs
     * @param percentSpeed    The speed of the robot from 0.0 to 1.0
     */
    public TeleopSwerve(SwerveDrive Swerve, CommandXboxController controller, int translationAxis, int strafeAxis,
            int rotationAxis, boolean fieldRelative, boolean openLoop) {
        m_Swerve = Swerve;
        addRequirements(m_Swerve);

        m_percentSpeed = NORMAL_SPEED;

        m_controller = controller;
        m_translationAxis = translationAxis;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;

        m_rotatePositions = new RotatePositions();
        m_rotationMathUtility = new RotationMathUtility();
    }

    @Override
    public void execute() {
        double rAxis;
        // Gets the inputs from the joysticks
        double yAxis = -m_controller.getHID().getRawAxis(m_translationAxis);
        double xAxis = -m_controller.getHID().getRawAxis(m_strafeAxis);
        // TODO REVIEW BEFORE ENABLING SINCE Y() was used by drive to release cone/cube
        // if (m_controller.getHID().getYButton()) {
        //     rAxis = m_rotationMathUtility.rotationSpeed(m_Swerve.getPose(), m_rotatePositions.getPose2dForRotation(), m_Swerve.getHeadingDegrees());
        // }
        // else{
            rAxis = -m_controller.getHID().getRawAxis(m_rotationAxis);
        // }

        // Applies a deadband to the values
        yAxis = MathUtil.applyDeadband(yAxis, STICK_DEAD_BAND);
        xAxis = MathUtil.applyDeadband(xAxis, STICK_DEAD_BAND);
        rAxis = MathUtil.applyDeadband(rAxis, STICK_DEAD_BAND);

        // Reduces the speed of the robot based on m_percentSpeed
        double m_throttle = yAxis * m_percentSpeed;
        double m_strafe = xAxis * m_percentSpeed;
        double m_rotation = rAxis * m_percentSpeed;

        m_Swerve.drive(m_throttle, m_strafe, m_rotation, m_fieldRelative, m_openLoop);
    }

    public void toggleSpeed() {
        if (m_percentSpeed == SLOW_SPEED) {
            m_percentSpeed = NORMAL_SPEED;
        } else {
            m_percentSpeed = SLOW_SPEED;
        }
        Logger.getInstance().recordOutput("TeleopSwerve/percentSpeed", m_percentSpeed);
    }
}
