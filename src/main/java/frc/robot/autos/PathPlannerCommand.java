package frc.robot.autos;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.FieldCentricOffset;
import frc.robot.utilities.SwerveAutoBuilder;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * <h3>PathPlannerCommand<h3>
 * 
 */
public class PathPlannerCommand extends SequentialCommandGroup {

    private static final double MAX_ACCELERATION = 2.5;
    private static final double MAX_VELOCITY = 1.0;

    /**
     * <h3>PathPlannerCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     * @param postCommand a command that is run after the path completes
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap, Command postCommand) {
        addRequirements(s_Swerve);

        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(pathName);
        if(pathConstraints == null) {
            pathConstraints = new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION);
        }

        // Load Path Group trajectory to follow.  All units in meters.
        List<PathPlannerTrajectory> loadPathGroup = PathPlanner.loadPathGroup(pathName, 
             false, pathConstraints);

        SwerveAutoBuilder autoBuilder = 
            new SwerveAutoBuilder(
                s_Swerve::getPose, //Using Pose Swerve estimator
                s_Swerve::resetOdometry, //pose2D consumer, used to reset odometry at beginning of zero
                SwerveDrive.getSwerveKinematics(),
                new PIDConstants(1.4, 0.0, 0.0), //PID constants to correct for translation error (X and Y)
                //new PIDConstants(1.0, 0.0, 0.0), //PID constants to correct for rotation error (used to create the rotation controller)
                new PIDConstants(1.9, 0.0, 0.0), //PID constants to correct for rotation error (used to create the rotation controller)
                s_Swerve::setSwerveModuleStates,
                eventCommandMap, 
                true, // TODO Should the path be automatically mirrored depending on alliance color
                s_Swerve);
        // creates a command based on the path group
        Command swerveControllerCommand = autoBuilder.fullAuto(loadPathGroup);
        addCommands(
            // TODO: Use april tags to help set this
            new InstantCommand(() -> FieldCentricOffset.getInstance().setOffset(loadPathGroup.get(0).getInitialHolonomicPose().getRotation().getDegrees())),
            swerveControllerCommand
        );
        if (postCommand != null) {
            addCommands(postCommand);
        }
    }

    /**
     * <h3>PathPlannerCommand</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param s_Swerve the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        this(s_Swerve, pathName, eventCommandMap, null);
    }
}