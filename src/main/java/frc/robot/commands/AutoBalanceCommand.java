package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/**
 * <h3>AutoBalanceCommand</h3>
 * 
 * Balances the robot on the Charging Station
 * 
 */
public class AutoBalanceCommand extends CommandBase {
    
    private final double MAX_SPEED = 0.2;
    private final double STRAFE = 0.0;
    private final double ROTATION = 0.0;
    private final boolean IS_FIELD_RELATIVE = true;
    private final boolean IS_OPEN_LOOP = true;

    private SwerveDrive m_swerveDrive;

    private double m_robotPitchInDegrees;
    private double m_throttle;

    /**
     * <h3>AutoBalanceCommand</h3>
     * 
     * Balances the robot on the Charging Station
     * 
     * @param swerveDrive Drive subsystem
     */
    public AutoBalanceCommand(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
        m_robotPitchInDegrees = 0.0;
        m_throttle = 0.0;
    }

    @Override
    public void execute() {
        // Get pitch in degrees from swerve drive subsystem
        m_robotPitchInDegrees = m_swerveDrive.getPitch().getDegrees();

        // Gets percentage of max speed to set swerve drive to
        double tempSpeed = 0.0;
        tempSpeed = MathUtil.clamp(m_swerveDrive.getAutoPitchController().calculate(m_robotPitchInDegrees/15, 0), -1, 1);

        // Logs values to advantage kit
        Logger.getInstance().recordOutput("AutoBalanceCommand/RobotPitch", m_robotPitchInDegrees);
        Logger.getInstance().recordOutput("AutoBalanceCommand/Speed", tempSpeed);
        
        // Sets drive to throttle
        m_throttle = tempSpeed * MAX_SPEED;
        m_swerveDrive.drive(m_throttle, STRAFE, ROTATION, IS_FIELD_RELATIVE, IS_OPEN_LOOP);
    }

    @Override
    public boolean isFinished() {
        return false; // Shouldn't end until switched to teleop
    }

}