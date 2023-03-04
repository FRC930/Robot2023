package frc.robot.autos;

import java.util.*;

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
                    SmartDashboard.putNumberArray("PathPlanner/DesiredPose", LogUtil.toPoseArray2d(targetPose));
                },
                null, // logSetPoint

                null // loggError
                // TODO  how to set default log error

        );
        SmartDashboard.putData("PP_Field", pp_field2d);
        //Subsystems used by auto commands
        SwerveDrive s_SwerveDrive = (SwerveDrive) subsystemMap.get(subNames.SwerveDriveSubsystem.toString());
        
        //Autonomous Commands
        TaxiOneBall taxiOneBall = new TaxiOneBall( s_SwerveDrive);
        Command taxiOneBallAutoBuildCommand = new PathPlannerCommand(s_SwerveDrive, "TaxiOneBall", eventCommandMap);
        Command MidScoreEngageCommand = new PathPlannerCommand(s_SwerveDrive, "MidScoreEngage", eventCommandMap, 
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command BumpSide1ConeCommand = new PathPlannerCommand(s_SwerveDrive, "BumpSide1Cone", eventCommandMap);
        Command BumpSide2ConeCommand = new PathPlannerCommand(s_SwerveDrive, "BumpSide2Cone", eventCommandMap);
        Command MidChargingStationCommand = new PathPlannerCommand(s_SwerveDrive, "MidChargingStation", eventCommandMap, 
            new AutoBalanceCommand(s_SwerveDrive));
        Command NoBump1ConeCommand = new PathPlannerCommand(s_SwerveDrive, "NoBump1Cone", eventCommandMap);
        Command NoBump2ConeCommand = new PathPlannerCommand(s_SwerveDrive, "NoBump2Cone", eventCommandMap);
        Command MidPickUpEngageCommand = new PathPlannerCommand(s_SwerveDrive, "MidPickUpEngage", eventCommandMap,
             new AutoBalanceCommand(s_SwerveDrive));
        Command ScoreHighConeCommand = new PathPlannerCommand(s_SwerveDrive, "ScoreHighCone", eventCommandMap);
        Command ScoreMidConeCommand = new PathPlannerCommand(s_SwerveDrive, "ScoreMidCone", eventCommandMap);
        Command ThreeConeNoBumpCommand = new PathPlannerCommand(s_SwerveDrive, "ThreeConeNoBump", eventCommandMap);
        Command ThreeConeBumpCommand = new PathPlannerCommand(s_SwerveDrive, "ThreeConeBump", eventCommandMap);
        Command TwoConeBalanceNoBumpCommand = new PathPlannerCommand(s_SwerveDrive, "TwoConeBalanceNoBump", eventCommandMap,
             new AutoBalanceCommand(s_SwerveDrive, true));   
        Command TwoConeEngageBumpCommand = new PathPlannerCommand(s_SwerveDrive, "TwoConeEngageBump", eventCommandMap, 
            new AutoBalanceCommand(s_SwerveDrive, true));
        Command MidScoreTwoEngageCommand = new PathPlannerCommand(s_SwerveDrive, "MidScoreTwoEngage", eventCommandMap, 
            new AutoBalanceCommand(s_SwerveDrive, true));
        
    


        // Adding options to the chooser in Shuffleboard/smartdashboard
        Boolean isBlue = (DriverStation.getAlliance() == Alliance.Blue);
        m_chooser.setDefaultOption("None", null);
        m_chooser.addOption("Taxi One Ball", taxiOneBall);
        m_chooser.addOption("taxiOneBallAutoBuild", taxiOneBallAutoBuildCommand);
        m_chooser.addOption("ScoreHighCone", ScoreHighConeCommand);
        m_chooser.addOption("ScoreMidCone", ScoreMidConeCommand);

        m_chooser.addOption("TwoConeEngageBump", TwoConeEngageBumpCommand);
        m_chooser.addOption("ThreeConeBump", ThreeConeBumpCommand);
        m_chooser.addOption("twoConeEngageNoBump",TwoConeBalanceNoBumpCommand);
        m_chooser.addOption("TwoConeBalanceNoBump", TwoConeBalanceNoBumpCommand);
        m_chooser.addOption("MidScoreEngage", MidScoreEngageCommand);
        m_chooser.addOption("BumpSide1Cone", BumpSide1ConeCommand);
        m_chooser.addOption("BumpSide2Cone", BumpSide2ConeCommand);
        m_chooser.addOption("MidChargingStation", MidChargingStationCommand);
        m_chooser.addOption("NoBump1Cone", NoBump1ConeCommand);
        m_chooser.addOption("NoBump2Cone", NoBump2ConeCommand);
        m_chooser.addOption("MidPickUpEngage",MidPickUpEngageCommand);
        m_chooser.addOption("ThreeConeNoBump",ThreeConeNoBumpCommand);
        m_chooser.addOption("MidScoreTwoEngage", MidScoreTwoEngageCommand);




        
        
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