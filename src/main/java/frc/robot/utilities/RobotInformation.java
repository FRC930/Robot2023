package frc.robot.utilities;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotInformation {

    private static final String WHICH_ROBOT_PREFERENCE_KEY = "WHICH_ROBOT";

    //sets the whichrobot input to which ever robot name it is  
    public enum WhichRobot {
      COMPETITION_ROBOT,
      PRACTICE_ROBOT;
      public static WhichRobot getEnum(String s){
        if(COMPETITION_ROBOT.name().equals(s)){
            return COMPETITION_ROBOT;
        }else if(PRACTICE_ROBOT.name().equals(s)){
            return PRACTICE_ROBOT;
        } else {
          WhichRobot[] nn = WhichRobot.values();
          throw new IllegalArgumentException("No WhichRobot Enum specified for this string: "+s
            +" Set robot Preference "+WHICH_ROBOT_PREFERENCE_KEY+" correctly to one these values: "+Arrays.toString(nn)
            +" Using SmartDashBoard!");
        }
      }
    }

    private SwerveModuleConstants m_FrontLeft;
    private SwerveModuleConstants m_FrontRight;
    private SwerveModuleConstants m_BackLeft;
    private SwerveModuleConstants m_BackRight;
    private WhichRobot m_WhichRobot;

    /**
     * <h3> RobotInformation</h3>
     * 
     * puts which robot on smartdashboard
     * 
     * @param whichRobot
     * @param frontLeft
     * @param frontRight
     * @param backLeft
     * @param backRight
     */
    public RobotInformation(WhichRobot whichRobot, 
            SwerveModuleConstants frontLeft,
            SwerveModuleConstants frontRight, 
            SwerveModuleConstants backLeft,
            SwerveModuleConstants backRight) {
        m_WhichRobot = whichRobot;
        SmartDashboard.putString("RobotInformation/RobotConfiguration", whichRobot.toString());
        m_FrontLeft = frontLeft;
        m_FrontRight = frontRight;
        m_BackLeft = backLeft;
        m_BackRight = backRight;
    }

    /**
     * <h3> whichrobotConFiguzRobot</h3>
     * 
     * returns whether its the competition robot or practice robot.
     * 
     * @return m_whichRobot
     */
    public WhichRobot whichRobotConfiguRobot() {
      return m_WhichRobot;
    }

    /**
     * <h3>getfrontLeft</h3>
     * 
     * returns front left Module
     * 
     * @return m_frontLeft
     */
    public SwerveModuleConstants getFrontLeft() {
      return m_FrontLeft;
    }

    /**
     * <h3>getFrontRight</h3>
     * 
     * returns front Right module
     *
     * @return m_frontRight
     */
    public SwerveModuleConstants getFrontRight() {
      return m_FrontRight;
    }

    /**
     * <h3> getBackLeft</h3>
     * 
     * returns back left module
     * 
     * @return m_backLeft
     */
    public SwerveModuleConstants getBackLeft() {
      return m_BackLeft;
    }

    /**
     * <h3> getBackRight</h3>
     * 
     * returns back right module
     * 
     * @return m_backRight
     */
    public SwerveModuleConstants getBackRight() {
      return m_BackRight;
    }

    /**
     * <h3>queryWhichRobotUsingPreferances</h3>
     *
     *  gets the prefered robot and returns it
     * 
     * @return WhichRobot
     */
    public static WhichRobot queryWhichRobotUsingPreferences() {
      // https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/smartdashboard-intro.html
      // To set value must run SmartDashboard when robot/simulation is connected.
      // View->Add...->Robot Preferences  
      // Add WHICH_ROBOT (string) value COMPETITION_ROBOT or PRACTICE_ROBOT
      String whichRobotString = Preferences.getString(WHICH_ROBOT_PREFERENCE_KEY, WhichRobot.PRACTICE_ROBOT.toString());
      return WhichRobot.getEnum(whichRobotString);
    }
}