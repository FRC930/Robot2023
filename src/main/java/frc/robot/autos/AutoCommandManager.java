package frc.robot.autos;

import java.util.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.*;
import frc.robot.utilities.LogUtil;

/**
 * <h3>AutonomouseCommandManager</h3>
 * 
 * Manages the autonomous paths by creating an instance of them and putting them
 * into the Shuffleboard.
 */
public class AutoCommandManager {
    HashMap<String, Subsystem> subsystemMap = new HashMap<String, Subsystem>();
    private Field2d pp_field2d = new Field2d();

    public static enum subNames {
        SwerveDriveSubsystem("SwerveDrive");

        final String m_name;

        subNames(String name) {
            m_name = name;
        }
    }

    /**
     * Adds a subbsystem to the subystem map
     *
     * @param SubNames
     * @param subsbystem
     */
    public void addSubsystem(subNames SubNames, Subsystem subsystem) {
        subsystemMap.put(SubNames.toString(), subsystem);
    }

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();
    
    /**
     * <h3>initCommands</h3>
     * 
     * Creates instances of each autonomous path command
     * 
     * @param eventCommandMap a command that uses strings to returna command that we want to execute at a marker
     */
    public void initCommands(Map<String, Command> eventCommandMap) {
        // Setup Logging for PathPlanner (for Ghost image)
        // NOTE: Make sure same as setting correct PPSwerveControllerCommand (ours or theirs)
        PPSwerveControllerCommand.setLoggingCallbacks(
                null, // logACtiveTRajectory
                // (PathPlannerTrajectory activeTrajectory) -> {
                //     // Log current trajectory
                //     pp_field2d.getObject("traj").setTrajectory(activeTrajectory);
                // },
                (Pose2d targetPose) -> {
                    // Log target pose
                    // TODO May not want both pose and trajectory
                    pp_field2d.setRobotPose(targetPose);
                    // May just want dashboard not on field2d
                    SmartDashboard.putNumberArray("PathPlanner/DesiredPose", LogUtil.toPoseArray2d(targetPose));
                },
                null, // logSetPoint
                // (ChassisSpeeds setpointSpeeds) -> {
                //     // Log setpoint ChassisSpeeds
                // },
                null // loggError
                // TODO  how to set default log error
                // (Translation2d translationError, Rotation2d rotationError) ->  {
                //     PPSwerveControllerCommand.defaultLogError(translationError, rotationError);
                // }
            );
        SmartDashboard.putData("PP_Field", pp_field2d);
        //Subsystems used by auto commands
        SwerveDrive s_SwerveDrive = (SwerveDrive) subsystemMap.get(subNames.SwerveDriveSubsystem.toString());
        
        //Autonomous Commands
        TaxiOneBall taxiOneBall = new TaxiOneBall( s_SwerveDrive);
        Command taxiOneBallAutoBuildCommand = new PathPlannerCommand(s_SwerveDrive, "TaxiOneBall", eventCommandMap);
        Command BlueLeftBalanceCommand = new PathPlannerCommand(s_SwerveDrive, "BlueLeftConeBalance", eventCommandMap);
        Command ChargeStationcommand = new PathPlannerCommand(s_SwerveDrive, "ChargingStation", eventCommandMap);
        Command MiddleCubeEngagecommand = new PathPlannerCommand(s_SwerveDrive, "MiddleCubeEngage", eventCommandMap);
        Command BlueRightCommand = new PathPlannerCommand(s_SwerveDrive, "BlueRight", eventCommandMap);
        Command BlueLeftCone = new PathPlannerCommand(s_SwerveDrive, "BlueLeftCone", eventCommandMap);
        //Adding options to the chooser
        m_chooser.setDefaultOption("None", null);
        m_chooser.addOption("Taxi One Ball", taxiOneBall);
        m_chooser.addOption("taxiOneBallAutoBuild", taxiOneBallAutoBuildCommand);
        m_chooser.addOption("score cone grab cone balance", BlueLeftBalanceCommand);
        m_chooser.addOption("Engage Charging Station", ChargeStationcommand);
        m_chooser.addOption("MiddleCubeEngagecommand", MiddleCubeEngagecommand);
        m_chooser.addOption("BlueRightCommand", BlueRightCommand);
        m_chooser.addOption("BlueLeftCone", BlueLeftCone);
        //adding chooser to dashboard
        SmartDashboard.putData("Auto choices", m_chooser);
    }
    /**
     *
     * Gets the autonomous path that is selected in the Shuffleboard
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

}
