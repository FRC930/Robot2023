package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * <h3>PPSwerveControllerCommand</h3>
 * 
 * Custom PathPlanner version of SwerveControllerCommand to log desired pose and expose trajectory and pose
 */
public class PPSwerveControllerCommand extends CommandBase {
  private final Timer timer = new Timer();
  protected  PathPlannerTrajectory m_trajectory;  // Need protected for can be call from extended object.
  protected final Supplier<Pose2d> m_poseSupplier; // Need protected for can be call from extended object.
  private final SwerveDriveKinematics m_kinematics;
  private final PPHolonomicDriveController m_controller;
  private final Consumer<SwerveModuleState[]> m_outputModuleStates;
  private final Consumer<ChassisSpeeds> m_outputChassisSpeeds;
  private final boolean m_useKinematics;
  private final boolean m_useAllianceColor;
  private final Field2d field = new Field2d();

  private PathPlannerTrajectory transformedTrajectory;

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but ChassisSpeeds from the position
   * controllers which need to be converted to module states and put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputChassisSpeeds The field relative chassis speeds output consumer.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
  public PPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      boolean useAllianceColor,
      Subsystem... requirements) {
    m_trajectory = trajectory;
    m_poseSupplier = poseSupplier;
    m_controller = new PPHolonomicDriveController(xController, yController, rotationController);
    m_outputChassisSpeeds = outputChassisSpeeds;
    m_outputModuleStates = null;
    m_kinematics = null;
    m_useKinematics = false;
    m_useAllianceColor = useAllianceColor;

    addRequirements(requirements);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but ChassisSpeeds from the position
   * controllers which need to be converted to module states and put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputChassisSpeeds The field relative chassis speeds output consumer.
   * @param requirements The subsystems to require.
   */
  public PPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      Subsystem... requirements) {
    this(
        trajectory,
        poseSupplier,
        xController,
        yController,
        rotationController,
        outputChassisSpeeds,
        true,
        requirements);
  }

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
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
  public PPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      boolean useAllianceColor,
      Subsystem... requirements) {
    m_trajectory = trajectory;
    m_poseSupplier = poseSupplier;
    m_kinematics = kinematics;
    m_controller = new PPHolonomicDriveController(xController, yController, rotationController);
    m_outputModuleStates = outputModuleStates;
    m_outputChassisSpeeds = null;
    m_useKinematics = true;
    m_useAllianceColor = useAllianceColor;

    addRequirements(requirements);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  public PPSwerveControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      PIDController rotationController,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Subsystem... requirements) {
    this(
        trajectory,
        poseSupplier,
        kinematics,
        xController,
        yController,
        rotationController,
        outputModuleStates,
        true,
        requirements);
  }

  @Override
  public void initialize() {
    if (m_useAllianceColor && m_trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              m_trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = m_trajectory;
    }

    SmartDashboard.putData("PPSwerveControllerCommand_field", this.field);
    this.field.getObject("traj").setTrajectory(transformedTrajectory);

    this.timer.reset();
    this.timer.start();

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);

    Pose2d currentPose = m_poseSupplier.get();
    //this.field.setRobotPose(currentPose); // FRC930 commented out
    Pose2d desiredPose;
    PathPlannerServer.sendPathFollowingData(
        desiredPose = new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
        currentPose);
    this.field.setRobotPose(desiredPose); // FRC930 set desiredpose not currentpos
    
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand_xError", currentPose.getX() - desiredState.poseMeters.getX());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand_yError", currentPose.getY() - desiredState.poseMeters.getY());
    SmartDashboard.putNumber(
        "PPSwerveControllerCommand_rotationError",
        currentPose.getRotation().getRadians() - desiredState.holonomicRotation.getRadians());

    ChassisSpeeds targetChassisSpeeds = m_controller.calculate(currentPose, desiredState);

    if (m_useKinematics) {
      SwerveModuleState[] targetModuleStates =
          m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

      m_outputModuleStates.accept(targetModuleStates);
    } else {
      m_outputChassisSpeeds.accept(targetChassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted) {
      if (m_useKinematics) {
        m_outputModuleStates.accept(
            m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
      } else {
        m_outputChassisSpeeds.accept(new ChassisSpeeds());
      }
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }
}
