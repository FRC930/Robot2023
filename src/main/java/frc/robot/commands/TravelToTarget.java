package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class TravelToTarget extends SequentialCommandGroup {
    
    private SwerveDrive m_swerveDrive;

/**
 * <h3>Travel</h3>
 * 
 * Travel moves the robot from a current pose to a set target during telep
 *  
 * @param targetPose The desired position
 * @param swerveDrive The swerve drive subsytem
 *
 */
    public TravelToTarget(Pose2d targetPose, SwerveDrive swerveDrive){
        m_swerveDrive = swerveDrive;
        addRequirements(swerveDrive);

        PIDController thetaController = m_swerveDrive.getAutoThetaController();

        //Automates a path using our drive command and PathPlanner
        PPSwerveControllerCommandWithIsFinish swerveControllerCommand =
        new PPSwerveControllerCommandWithIsFinish(targetPose,
            null, // set to NULL given need to set trajectory once command is initiated not created!
             m_swerveDrive::getPose, 
            SwerveDrive.getSwerveKinematics(),
            m_swerveDrive.getAutoXController(),
            m_swerveDrive.getAutoYController(),
            thetaController,
            m_swerveDrive::setSwerveModuleStates,
            false,  // MUST BE false of NPE due to reversing path and I don't think want to reverse anyhow
            m_swerveDrive);
        addCommands(swerveControllerCommand);
    }
}
