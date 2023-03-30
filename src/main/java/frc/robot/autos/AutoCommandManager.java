package frc.robot.autos;

import java.util.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.PPSwerveControllerCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.utilities.CommandFactoryUtility;
import frc.robot.utilities.LogUtil;

/**
 * <h3>AutonomouseCommandManager</h3>
 * 
 * Manages the autonomous paths by creating an instance of them and putting them
 * into the Shuffleboard.
 */
public class AutoCommandManager {
    public AutoCommandManager() {
        SmartDashboard.putBoolean(this.getClass().getSimpleName()+"/CanTunePIDValues", TUNE_PID);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kPX", kPXController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kIX", kIXController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kDX", kDXController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kPY", kPYController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kIY", kIYController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kDY", kDYController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kPTheta", kPThetaController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kITheta", kIThetaController);
        SmartDashboard.putNumber(this.getClass().getSimpleName()+"/kDTheta", kDThetaController);
    }

    HashMap<String, Subsystem> subsystemMap = new HashMap<String, Subsystem>();
    private Field2d pp_field2d = new Field2d();

    public static enum subNames {
        SwerveDriveSubsystem("SwerveDrive"),
        ElevatorSubsystem("Elevator"),
        ArmSubsystem("Arm"),
        ManipulatorSubsystem("Manipulator");

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

    private static final boolean TUNE_PID = true;
    //TODO TUNE FOR GHOST
    public static final double kPXController = usePIDValueOrTune("kPX",3.596); //0.076301;
    public static final double kIXController = usePIDValueOrTune("kIX",0.0);; 
    public static final double kDXController = usePIDValueOrTune("kDX",0.0);; 
    public static final double kPYController = usePIDValueOrTune("kPY",3.596); //0.076301;
    public static final double kIYController = usePIDValueOrTune("kIY",0.0); 
    public static final double kDYController = usePIDValueOrTune("kDY",0.0);  
    public static final double kPThetaController = usePIDValueOrTune("kPTheta",3.2);
    public static final double kIThetaController = usePIDValueOrTune("kITheta",0.0);
    public static final double kDThetaController = usePIDValueOrTune("kDTheta",0.0);
    
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
                null, 

                (Pose2d targetPose) -> {
                    // Log target pose
                    // TODO May not want both pose and trajectory
                    pp_field2d.setRobotPose(targetPose);
                    // May just want dashboard not on field2d
                    Logger.getInstance().recordOutput("PathPlanner/DesiredPose",targetPose);
                },
                null, // logSetPoint

                null // loggError
                // TODO  how to set default log error

        );
        SmartDashboard.putData("PP_Field", pp_field2d);
        //Subsystems used by auto commands
        SwerveDrive s_SwerveDrive = (SwerveDrive) subsystemMap.get(subNames.SwerveDriveSubsystem.toString());
        ElevatorSubsystem m_elevatorSubsystem = (ElevatorSubsystem) subsystemMap.get(subNames.ElevatorSubsystem.toString());
        ArmSubsystem m_armSubsystem = (ArmSubsystem) subsystemMap.get(subNames.ArmSubsystem.toString());
        ManipulatorSubsystem m_manipulatorSubsystem = (ManipulatorSubsystem) subsystemMap.get(subNames.ManipulatorSubsystem.toString());
        
        //Autonomous Commands
        Command MidScoreEngageCommand = new PathPlannerCommand(s_SwerveDrive, "MidScoreEngage", eventCommandMap, 
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command MidChargingStationCommand = new PathPlannerCommand(s_SwerveDrive, "MidChargingStation", eventCommandMap, 
            new AutoBalanceCommand(s_SwerveDrive));
        Command ScoreHighConeCommand = new PathPlannerCommand(s_SwerveDrive, "ScoreHighCone", eventCommandMap);
        Command One_ConeCubeNoBump = new PathPlannerCommand(s_SwerveDrive, "1_ConeCubeNoBump", eventCommandMap);
        Command Two_ConeCubeBalanceNoBump = new PathPlannerCommand(s_SwerveDrive, "2_ConeCubeBalanceNoBump", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command Three_ConeCubeBalanceBump = new PathPlannerCommand(s_SwerveDrive, "3_ConeCubeBalanceBump", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, false));
        Command Four_ConeCubeBump = new PathPlannerCommand(s_SwerveDrive, "4_ConeCubeBump", eventCommandMap);

        Command One_ConeCubeNoBumpV2 = new PathPlannerCommand(s_SwerveDrive, "1_ConeCubeNoBumpV2", eventCommandMap);
        Command Two_ConeCubeBalanceNoBumpV2 = new PathPlannerCommand(s_SwerveDrive, "2_ConeCubeBalanceNoBumpV2", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, false));
        Command Three_ConeCubeBalanceBumpV2 = new PathPlannerCommand(
            CommandFactoryUtility.createAutoScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem),
            s_SwerveDrive, "3_ConeCubeBalanceBumpV3", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command Four_ConeCubeBumpV2 = new PathPlannerCommand(
            CommandFactoryUtility.createAutoScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem),
            s_SwerveDrive, "4_ConeCubeBumpV3", eventCommandMap);
        
        Command BumpConeSConeSCubeEngaged = new PathPlannerCommand(s_SwerveDrive, "BumpConeSConeSCubeEngaged", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command NoBumpConeSConeSCubeEngaged = new PathPlannerCommand(s_SwerveDrive, "NoBumpConeSConeSCubeEngaged", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command NoBumpConeSConeSCubeEngageV2 = new PathPlannerCommand(s_SwerveDrive, "NoBumpConeSConeSCubeEngageV2", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, false));
        Command NoBumpConeSConeSCubeS = new PathPlannerCommand(
             CommandFactoryUtility.createAutoScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem),
            s_SwerveDrive, "NoBumpConeSConeSCubeSV3", eventCommandMap);
        Command NoBumpConeSCubeSCubeEngageV2 = new PathPlannerCommand(
            CommandFactoryUtility.createAutoScoreHighCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem),
            s_SwerveDrive, "NoBumpConeSCubeSCubeEngageV3", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command Three_ConeCubeNoBalanceBumpV2 = new PathPlannerCommand(s_SwerveDrive, "3_ConeCubeNoBalanceBumpV2", eventCommandMap);
        Command NoBumpConeSCubeSV2 = new PathPlannerCommand(s_SwerveDrive, "NoBumpConeSCubeSV2", eventCommandMap);
        Command NoBumpMConeSMCubeSCubeSV3 = new PathPlannerCommand(
            CommandFactoryUtility.createAutoScoreMidCommand(m_elevatorSubsystem, m_armSubsystem, m_manipulatorSubsystem), 
            s_SwerveDrive, "NoBumpMConeSMCubeSCubeSV3", eventCommandMap);

        Command NoBumpMConeSMCubeSEngageV3 = new PathPlannerCommand(s_SwerveDrive, "NoBumpMConeSMCubeSEngageV3", eventCommandMap,
            new AutoBalanceCommand(s_SwerveDrive, false));    
        // Adding options to the chooser in Shuffleboard/smartdashboard
        Boolean isBlue = (DriverStation.getAlliance() == Alliance.Blue);
        m_chooser.setDefaultOption("None", null);

        //commented *** to show that the path is untested
        //m_chooser.addOption("BumpConeSConeSCubeEngaged***", BumpConeSConeSCubeEngaged);
        //m_chooser.addOption("NoBumpConeSConeSCubeEngaged***", NoBumpConeSConeSCubeEngaged);

        //m_chooser.addOption("1_NoBumpConeSCubeS", One_ConeCubeNoBump);
        //m_chooser.addOption("2_NoBumpConeSCubeNSEngaged***", Two_ConeCubeBalanceNoBump);
        //m_chooser.addOption("3_BumpConeSCubeNSEngaged", Three_ConeCubeBalanceBump);
        //m_chooser.addOption("4_BumpConeSCubeS***", Four_ConeCubeBump);

        // m_chooser.addOption("1_NoBumpConeSCubeSV2***", One_ConeCubeNoBumpV2);
        // m_chooser.addOption("2_NoBumpConeSCubeNSEngagedV2***", Two_ConeCubeBalanceNoBumpV2);
        m_chooser.addOption("3_BumpConeSCubeNSEngagedV2", Three_ConeCubeBalanceBumpV2);
        //m_chooser.addOption("*****3_BumpConeSCubeNSV2******", Three_ConeCubeNoBalanceBumpV2);
        m_chooser.addOption("***4_BumpConeSCubeSV2***", Four_ConeCubeBumpV2);
       // m_chooser.addOption("***NoBumpConeSCubeSV2***", NoBumpConeSCubeSV2);

        //m_chooser.addOption("ScoreHighCone", ScoreHighConeCommand);

        //m_chooser.addOption("MidScoreEngage", MidScoreEngageCommand);
        //m_chooser.addOption("MidChargingStation", MidChargingStationCommand);
       // m_chooser.addOption("NoBumpConeSConeSCubeEngageV2", NoBumpConeSConeSCubeEngageV2);
        m_chooser.addOption("NoBumpConeSConeSCubeS", NoBumpConeSConeSCubeS);
        //m_chooser.addOption("NoBumpConeSCubeSCubeEngageV2", NoBumpConeSCubeSCubeEngageV2);
        m_chooser.addOption("NoBumpMConeSMCubeSCubeSV3", NoBumpMConeSMCubeSCubeSV3);
        m_chooser.addOption("NoBumpMConeSMCubeSEngageV3", NoBumpMConeSMCubeSEngageV3);


        
        
        //Adding chooser to Shuffleboard/Smartdashboard
        SmartDashboard.putData("Auto choices", m_chooser);
    }

    /**
    * <h3>usePIDVauleOrTune</h3>
    *
    * manualy set a defautValue in robot container and if Tune_pid is true 
    * it gives pidValue with the key and defultvalue if false gives just the defaultValue
    * @param key
    * @param defaultValue
    * @return pidValue
    */
    public static double usePIDValueOrTune(String key, double defaultValue) {
        double pidValue;
        if(TUNE_PID) {
            pidValue = Preferences.getDouble(key, defaultValue);
        } else {
            pidValue = defaultValue;
        }
        // Save Value so don't need tuning (so value is saved)
        Preferences.setDouble(key, pidValue);     
        return pidValue;
    }
    
    /**
     *<h3> getAutonomousCommand</h3>

     * Gets the autonomous path that is selected in the Shuffleboard
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

}