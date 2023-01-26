package frc.robot.autos;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveDrive;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TaxiOneBall extends SequentialCommandGroup {
    public TaxiOneBall(SwerveDrive s_Swerve) {

        // An example trajectory to follow.  All units in meters.
        PathPlannerTrajectory pathPlannerExample = PathPlanner.loadPath("TaxiOneBall", 1, 2.5, false);
        Trajectory exampleTrajectory = pathPlannerExample;
        
        var thetaController = s_Swerve.getAutoThetaController();
        thetaController.enableContinuousInput(-180.0, 180.0); //-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand = 
            new PPSwerveControllerCommand(
                pathPlannerExample,
                s_Swerve::getPoseMeters,
                SwerveDrive.getSwerveKinematics(),
                s_Swerve.getAutoXController(),
                s_Swerve.getAutoYController(),
                thetaController,
                s_Swerve::setSwerveModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(pathPlannerExample.getInitialHolonomicPose())),
            swerveControllerCommand
        );
    }
}