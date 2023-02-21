package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PPSwerveControllerCommandWithIsFinish extends PPSwerveControllerCommand {


    // TODO tune the value
    private static final double DELTA_X = Units.inchesToMeters(3.15);
    private static final double DELTA_Y = Units.inchesToMeters(3.15);

    // TODO Passing or constanst from auto?
    private double MAX_SPEED = 3.0;
    private double MAX_ACCELERATION = 1.0;
    private Pose2d m_targetPose;

 /**
   * Extended PPSwerveControllerCommand to override IsFinished method to stop the robot.
   * 
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * 
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
    public PPSwerveControllerCommandWithIsFinish(Pose2d target,  PathPlannerTrajectory trajectory, Supplier<Pose2d> poseSupplier, SwerveDriveKinematics kinematics,
            PIDController xController, PIDController yController, PIDController rotationController,
            Consumer<SwerveModuleState[]> outputModuleStates, boolean useAllianceColor, Subsystem... requirements) {
        // useAllinceColor API so dont get NULL pointer on trajector
        super(trajectory, poseSupplier, kinematics, xController, yController, rotationController, outputModuleStates,
        useAllianceColor, requirements);
        m_targetPose = target;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = super.poseSupplier.get();
        double x1 = currentPose.getX();
        double y1 = currentPose.getY();
        double x2 = m_targetPose.getX();
        double y2 = m_targetPose.getY();
        Rotation2d angle = Rotation2d.fromDegrees(Math.atan2( y2 - y1, x2 - x1 ) * ( 180 / Math.PI ));

        // The pathpoint is the starting and ending positions of the robot
        PathPoint currentPathPoint = new PathPoint(currentPose.getTranslation(),angle);
        PathPoint targetPathPoint = new PathPoint(m_targetPose.getTranslation(),angle, m_targetPose.getRotation());
        
        //TODO May want to add a midpoint to avoid charging station (one to the left or right) 
        PathConstraints pathConstraints = new PathConstraints(MAX_SPEED, MAX_ACCELERATION);
        super.trajectory = PathPlanner.generatePath(pathConstraints, false, currentPathPoint, targetPathPoint);        
        super.initialize();
    }

    /**
     * <h3>isFinished<h3>
     * 
     * Stopping the command if the robot is within a range of the target
     * 
     */
    @Override
    public boolean isFinished() { 
        boolean isfinish = false;
        
        Pose2d currentPose = super.poseSupplier.get();
        if(currentPose != null) {
            Transform2d deltaPose = currentPose.minus(m_targetPose);
            // If the robot is within a range of the target only using X and Y coordinates the command finishes
            if(Math.abs(deltaPose.getX())<= DELTA_X 
                && Math.abs(deltaPose.getY())<= DELTA_Y) {
                isfinish = true;
            }  
        }
        return isfinish;
    }
}
