package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/**
 * <h3>SwerveXCommand</h3>
 * 
 * Locks the wheels in a X formation
 *
 */
public class SwerveLockCommand extends CommandBase{
    
    private SwerveDrive m_SwerveDrive;
    private boolean m_isOpenLoop;

    /**
     * <h3>SwerveXCommand</h3>
     * 
     * Locks the wheels in a X formation
     *
     * @param swerveDrive swerve drive
     * @param isOpenLoop If true, it doesn't use PID values to correct inputs
     * 
     */
    public SwerveLockCommand(SwerveDrive swerveDrive, boolean isOpenLoop) {
        m_SwerveDrive = swerveDrive;
        m_isOpenLoop = isOpenLoop;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        m_SwerveDrive.lockPose(m_isOpenLoop);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

}
