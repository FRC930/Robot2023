package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;

public class PPSwerveControllerCommandWithIsFinish extends PPSwerveControllerCommand {


    // TODO Passing or constanst from auto?
    private double MAX_SPEED = 3.0;
    private double MAX_ACCELERATION = 1.0;
    private Pose2d m_targetPose;

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
        Rotation2d curRotation = currentPose.getRotation();
        if(currentPose.getX() > m_targetPose.getX()) {
            // TODO fix not going right way???
            // TODO ACTUALLY currentpose not GIVING correect rotation!!!!
            curRotation= curRotation;
        } // TODO Acutally both it appear to go 90 both moving
        // The pathpoint is the starting and ending positions of the robot
        PathPoint currentPathPoint = new PathPoint(currentPose.getTranslation(),curRotation);
        PathPoint targetPathPoint = new PathPoint(m_targetPose.getTranslation(),m_targetPose.getRotation());

        // TODO REMOVE
        Transform2d deltaPose = currentPose.minus(m_targetPose);
        System.out.println("INIT Current:"+currentPose.getX()+"Delta:"+deltaPose.getX());

        PathConstraints pathConstraints = new PathConstraints(MAX_SPEED, MAX_ACCELERATION);
        // TODO remove commented code since can pass in two points directly
        //pathLists keeps track of the PathPoints
        // List<PathPoint> pathLists = new ArrayList<PathPoint>();
        // pathLists.add(currentPathPoint);
        // pathLists.add(targetPathPoint); 
        //creates the command goFromHereToThere using the generatePath method for PathPlanner
        //super.trajectory = PathPlanner.generatePath(pathConstraints, false, pathLists);        
        super.trajectory = PathPlanner.generatePath(pathConstraints, false, currentPathPoint, targetPathPoint);        
        Logger.getInstance().recordOutput("Debug/initpose",super.trajectory.getInitialHolonomicPose());
        super.initialize();
    }

    @Override
    public boolean isFinished() { 
        boolean isfinish = false;
        
        Pose2d currentPose = super.poseSupplier.get();
        if(currentPose != null) {
            Transform2d deltaPose = currentPose.minus(m_targetPose);
            // TODO finish this with other dimentions and remove println
            // TODO constant or pass in deltadiff number
            if(Math.abs(deltaPose.getX())<= 0.08 
            && Math.abs(deltaPose.getY())<= 0.08 
            //&& Math.abs(currentPose.getRotation().getDegrees()-m_targetPose.getRotation().getDegrees())<= 10
            ) {
                isfinish = true;
                System.out.println("STOPPED CurrentX:"+currentPose.getX()+"Delta:"+deltaPose.getX());
                System.out.println("STOPPED CurrentY:"+currentPose.getY()+"Delta:"+deltaPose.getY());
                System.out.println("STOPPED CurrentRotation2D:"+currentPose.getRotation().getDegrees()+"Target:"+m_targetPose.getRotation().getDegrees());
            }  else {
                System.out.println("CurrentX:"+currentPose.getX()+"Delta:"+deltaPose.getX());
                System.out.println("CurrentY:"+currentPose.getY()+"Delta:"+deltaPose.getY());
                System.out.println("CurrentRotation2D:"+currentPose.getRotation().getDegrees()+"Target:"+m_targetPose.getRotation().getDegrees());
            }
        }
        return isfinish;
    }

}
