package frc.robot.utilities;

import java.io.IOException;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import frc.robot.Robot;
import frc.robot.utilities.vision.estimation.CameraProperties;

public class CameraOnRobot {

    private final PhotonCamera m_PhotonCamera;
    private CameraProperties m_CameraProp;
    private final Transform3d m_RobotToCameraPose;
    private String m_CameraIpName;
    private String m_CameraName;  // TODO comment why keep

    
    /**
     * <h3>CameraOnRobot</h3>
     * 
     * Creates a new camera with all of its attributes
     * 
     * @param cameraName name of the camera ex: back camera
     * @param cameraIpName ip address of the camera
     * @param pipelineIndex photon pipeline number
     * @param portToForward port number to access the camera, local is 5800, camera is 580x, x = camera number
     * @param configFile config file of the camera
     * @param cameraXPosition difference in x position between the camera and the center of the robot 
     * @param cameraYPosition difference in y position between the camera and the center of the robot 
     * @param cameraZPosition difference in z position between the camera and the center of the robot 
     * @param cameraRotationRoll difference in roll rotation between the camera and the center of the robot 
     * @param cameraRotationPitch difference in pitch rotation between the camera and the center of the robot
     * @param cameraRotationYaw difference in yaw rotation between the camera and the center of the robot
     */
    public CameraOnRobot(String cameraName, 
                        String cameraIpName, 
                        int pipelineIndex,
                        int portToForward,
                        String configFile, 
                        int cameraResolutionWidth,
                        int cameraResolutionHeight,
                        double cameraXPosition,
                        double cameraYPosition,
                        double cameraZPosition,
                        double cameraRotationRoll,
                        double cameraRotationPitch,
                        double cameraRotationYaw) {

        m_PhotonCamera = new PhotonCamera(cameraName);
        m_PhotonCamera.setDriverMode(false);
        m_CameraIpName = cameraIpName;
        PortForwarder.add(portToForward, cameraIpName, 5800);
        m_PhotonCamera.setPipelineIndex(pipelineIndex);

        if (Robot.isSimulation()) { 
            // disable version check when running simulation 
            PhotonCamera.setVersionCheckEnabled(false); 
        }

        try {
            m_CameraProp = new CameraProperties(Filesystem.getDeployDirectory()+"/"+configFile, cameraResolutionWidth, cameraResolutionHeight);
        } catch (IOException e) {
            if(m_PhotonCamera == null) {
                m_CameraProp = CameraProperties.LL2_640_480();
            } 
            DriverStation.reportWarning("Unable to load configuration file for camera "+cameraIpName, e.getStackTrace());
        }
       
        m_RobotToCameraPose = new Transform3d(
            new Translation3d(
                cameraXPosition,
                cameraYPosition,
                cameraZPosition
            ),
            new Rotation3d(
                cameraRotationRoll,
                cameraRotationPitch,
                cameraRotationYaw
            )
        );
    }

    public String getCameraIpName() {
        return m_CameraIpName;
    }

    public void setM_CameraIpName(String m_CameraIpName) {
        this.m_CameraIpName = m_CameraIpName;
    }

    public PhotonCamera getPhotonCamera() {
        return m_PhotonCamera;
    }

    public CameraProperties getCameraProp() {
        return m_CameraProp;
    }

    public Transform3d getRobotToCameraPose() {
        return m_RobotToCameraPose;
    }

}