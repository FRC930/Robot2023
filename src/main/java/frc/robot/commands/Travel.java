package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveDrive;

public class Travel extends SequentialCommandGroup{
    
    private SwerveDrive m_swerveDrive;

    private double MAX_SPEED = 4.0;
    private double MAX_ACCELERATION = 2.0;
    private Pose2d target;
/**
 * <h3>Travel</h3>
 * 
 * Travel moves the robot from a current pose to a set target during telep
 * 
 * 
 * @param targetPose The wanted position
 * @param swervyDrive The swerve drive subsytem
 *
 * 
 * 
 */
    public Travel(Pose2d targetPose, SwerveDrive swervyDrive){
        m_swerveDrive = swervyDrive;
        addRequirements(swervyDrive);
        target = targetPose;
        Pose2d currentPose = m_swerveDrive.getPose();
        // The pathpoint is the starting and ending positions of the robot
        PathPoint currentPathPoint = new PathPoint(currentPose.getTranslation(),currentPose.getRotation());
        PathPoint targetPathPoint = new PathPoint(targetPose.getTranslation(),targetPose.getRotation());
        PathConstraints pathConstraints = new PathConstraints(MAX_SPEED, MAX_ACCELERATION);
        //pathLists keeps track of the PathPoints
        List<PathPoint> pathLists = new ArrayList<PathPoint>();
        pathLists.add(currentPathPoint);
        pathLists.add(targetPathPoint); 
        //creates the command goFromHereToThere using the generatePath method for PathPlanner
        PathPlannerTrajectory goFromHereToThere = PathPlanner.generatePath(pathConstraints, false, pathLists);
        var thetaController = m_swerveDrive.getAutoThetaController();

        //Automates a path using our drive command and PathPlanner
        PPSwerveControllerCommand swerveControllerCommand =
        new PPSwerveControllerCommand(
            goFromHereToThere,
            m_swerveDrive::getPoseMeters,
            SwerveDrive.getSwerveKinematics(),
            m_swerveDrive.getAutoXController(),
            m_swerveDrive.getAutoYController(),
            thetaController,
            m_swerveDrive::setSwerveModuleStates,
            m_swerveDrive);

            addCommands(
           //     new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                swerveControllerCommand
            );
        
    }

    
}
