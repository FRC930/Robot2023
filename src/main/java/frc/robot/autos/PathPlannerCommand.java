package frc.robot.autos;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.SwerveAutoBuilder;

import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
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
     */
    public PathPlannerCommand(SwerveDrive s_Swerve, String pathName, Map<String, Command> eventCommandMap) {
        addRequirements(s_Swerve);

        PathConstraints pathConstraints = PathPlanner.getConstraintsFromPath(pathName);
        if(pathConstraints == null) {
            //TODO add constraints
            pathConstraints = new PathConstraints(MAX_VELOCITY, MAX_ACCELERATION);
        }

        // Load Path Group trajectory to follow.  All units in meters.
        List<PathPlannerTrajectory> loadPathGroup = PathPlanner.loadPathGroup(pathName, 
             false, pathConstraints);
        
        var thetaController = s_Swerve.getAutoThetaController();
        thetaController.enableContinuousInput(-180.0, 180.0); //-Math.PI, Math.PI);

        SwerveAutoBuilder autoBuilder = 
            new SwerveAutoBuilder(
                s_Swerve::getPose, //Using Pose Swerve estimator
                s_Swerve::resetOdometry, //pose2D consumer, used to reset odometry at beginning of zero
                SwerveDrive.getSwerveKinematics(),
                getPIDConstants(s_Swerve.getAutoXController()), //PID constants to correct for translation error (X and Y)
                //s_Swerve.getAutoYController(), //NOTE:  This does not use a YPID it only has one for both X and Y
                getPIDConstants(thetaController), //PID constants to correct for rotation error (used to create the rotation controller)
                s_Swerve::setSwerveModuleStates,
                eventCommandMap, 
                false, // TODO Should the path be automatically mirrored depending on alliance color
                s_Swerve);
        // creates a command based on the path group
        Command swerveControllerCommand = autoBuilder.fullAuto(loadPathGroup);
        addCommands(
       //     new InstantCommand(() -> s_Swerve.resetOdometry(pathPlannerExample.getInitialHolonomicPose())),
            swerveControllerCommand
        );
    }
    /**
     * <h3>getPIDConstants</h3>
     * 
     * gets the PID constants from the PID controller
     * 
     * @param pidController 
     * @return returns the PID constants
     */
    private PIDConstants getPIDConstants(PIDController pidController) {
        return new PIDConstants(pidController.getP(), pidController.getI(), pidController.getD());
    }
}