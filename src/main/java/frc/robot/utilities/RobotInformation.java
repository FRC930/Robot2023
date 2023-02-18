package frc.robot.utilities;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotInformation {

    private static final String WHICH_ROBOT_PREFERENCE_KEY = "WHICH_ROBOT";

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
    private SwerveModuleConstants m_FrontRIght;
    private SwerveModuleConstants m_BackLeft;
    private SwerveModuleConstants m_BackRight;
    private WhichRobot m_WhichRobot;

    public RobotInformation(WhichRobot whichRobot, 
            SwerveModuleConstants frontLeft,
            SwerveModuleConstants frontRight, 
            SwerveModuleConstants backLeft,
            SwerveModuleConstants backRight) {
        m_WhichRobot = whichRobot;
        SmartDashboard.putString("RobotInformation/RobotConfiguration", whichRobot.toString());
        m_FrontLeft = frontLeft;
        m_FrontRIght = frontRight;
        m_BackLeft = backLeft;
        m_BackRight = backRight;
    }

    public WhichRobot whichRobotConfiguRobot() {
      return m_WhichRobot;
    }

    public SwerveModuleConstants getFrontLeft() {
      return m_FrontLeft;
    }

    public SwerveModuleConstants getFrontRight() {
      return m_FrontRIght;
    }

    public SwerveModuleConstants getBackLeft() {
      return m_BackLeft;
    }

    public SwerveModuleConstants getBackRight() {
      return m_BackRight;
    }

    public static WhichRobot queryWhichRobotUsingPreferences() {
      // https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/smartdashboard-intro.html
      // To set value must run SmartDashboard when robot/simulation is connected.
      // View->Add...->Robot Preferences  
      // Add WHICH_ROBOT (string) value COMPETITION_ROBOT or PRACTICE_ROBOT
      String whichRobotString = Preferences.getString(WHICH_ROBOT_PREFERENCE_KEY, WhichRobot.PRACTICE_ROBOT.toString());
      return WhichRobot.getEnum(whichRobotString);
    }
}