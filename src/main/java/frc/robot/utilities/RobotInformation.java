package frc.robot.utilities;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class RobotInformation {

    // SET MAC Address for competition robot
    public static final String COMPETIION_ROBOT_MAC_ADDRESS = "00-80-2F-18-15-63";
    private static String macAddress = RobotInformation.getMACAdress();
    private SwerveModuleConstants m_FrontLeft;
    private SwerveModuleConstants m_FrontRIght;
    private SwerveModuleConstants m_BackLeft;
    private SwerveModuleConstants m_BackRight;
    private boolean m_useCompetitionConfiguration;

    public RobotInformation(boolean useCompetitionConfiguration, 
            SwerveModuleConstants frontLeft,
            SwerveModuleConstants frontRight, 
            SwerveModuleConstants backLeft,
            SwerveModuleConstants backRight) {
        m_useCompetitionConfiguration = useCompetitionConfiguration;
        Logger.getInstance().recordOutput("Robot/UseCompetitionConfiguration", m_useCompetitionConfiguration);
        m_FrontLeft = frontLeft;
        m_FrontRIght = frontRight;
        m_BackLeft = backLeft;
        m_BackRight = backRight;
    }

    public boolean isCompetitionConfiguration() {
      return m_useCompetitionConfiguration;
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

    public static boolean queryIfCompetitionRobot(boolean default_to_competition_robot_in_simulation) {
      return Robot.isReal()?macAddress.equals(COMPETIION_ROBOT_MAC_ADDRESS):default_to_competition_robot_in_simulation;
    }
    // TODO Find a better solution to getting the MAC adress 
    //Doesn't appear to get the MAC adress when the RoboRio starts up
    public static String getMACAdress() {
      String address = getInternalMACAdress();
      int counter = 0;
      while(address.equals("unknown") && counter<100) {
        try {
          Thread.sleep(10);
          System.out.println("Counter = " + counter);
          counter++;
          address = getInternalMACAdress();
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }      
      return address;
    }
    public static String getInternalMACAdress() {
        String macAddress="unknown";
        InetAddress localHost;
        NetworkInterface ni;
        try {
          localHost = InetAddress.getLocalHost();
          ni = NetworkInterface.getByInetAddress(localHost);
          byte[] hardwareAddress = ni.getHardwareAddress();
          if(hardwareAddress != null){
            String[] hexadecimal = new String[hardwareAddress.length];
            for (int i = 0; i < hardwareAddress.length; i++) {
                hexadecimal[i] = String.format("%02X", hardwareAddress[i]);
            }
            macAddress = String.join("-", hexadecimal);
          }else{
            DriverStation.reportWarning("Unable to get MAC address (hardwareAdress is null)", Thread.currentThread().getStackTrace());
          }

        } catch (SocketException e) {
          DriverStation.reportWarning("Unable to get MAC address", e.getStackTrace());
        }catch (UnknownHostException e) {
          DriverStation.reportWarning("Unable to get MAC address (unknown host)", e.getStackTrace());
        }
        Logger.getInstance().recordOutput("Robot/MACAddress", macAddress);
        return macAddress;
      }
}