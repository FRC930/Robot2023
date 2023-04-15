package frc.robot.utilities;

import java.io.IOException;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



/**
 * <h3>OdometryUtility</h3>
 * 
 * OdometryUtility holds the cameras that we use for our aiming.
 */
public class OdometryUtility {
    // Used for simulation Must have photovision camera running network server (enable it)
    // IMPORTANT can not have turned on when on robot
    public static final boolean CONNECTED_PHOTOVISION_CAMERA = false;
    public static final String PHOTOVISION_NETWORK_SERVER = "10.9.30.35";

    //TODO Configure postitions of cameras
    /*
     * IMPORTANT CAMERA INFO
     * 
     * Most values need to be tested/checked and are different for each camera
     * The resolution, position, and rotation values will need the most tuning
     * Position and rotation are accounting for the difference between the camera and the center of the robot
     * 
     */

    // Back camera constants
    private static final String BACK_CAMERA_NAME = "Camera3"; 
    private static final String BACK_CAMERA_IP_NAME = "10.9.30.33";
    private static final int BACK_CAMERA_PIPELINE = 0;
    private static final int BACK_CAMERA_PORT_TO_FORWARD = 5803;
    private static final String BACK_CAMERA_CONFIG_FILE = "CameraConfigs/Camera3/config.json";
    private static final int BACK_CAMERA_RESOLUTION_WIDTH = 640;
    private static final int BACK_CAMERA_RESOLUTION_HEIGHT = 480;
    private static final double BACK_CAMERA_POSITION_X = Units.inchesToMeters(-10.223);
    private static final double BACK_CAMERA_POSITION_Y = Units.inchesToMeters(10.462226);
    private static final double BACK_CAMERA_POSITION_Z = Units.inchesToMeters(10.0);
    private static final double BACK_CAMERA_ROTATION_ROLL = Math.toRadians(0.0);
    private static final double BACK_CAMERA_ROTATION_PITCH = Math.toRadians(0.0);
    private static final double BACK_CAMERA_ROTATION_YAW = Math.toRadians(165.0);

    static final Transform3d robotToBackCam = 
        new Transform3d(
            new Translation3d(BACK_CAMERA_POSITION_X, BACK_CAMERA_POSITION_Y, BACK_CAMERA_POSITION_Z),
            new Rotation3d(BACK_CAMERA_ROTATION_ROLL,BACK_CAMERA_ROTATION_PITCH, BACK_CAMERA_ROTATION_YAW));


    //Left camera constants
    private static final String LEFT_CAMERA_NAME = "Camera2"; 
    private static final String LEFT_CAMERA_IP_NAME = "10.9.30.32";
    private static final int LEFT_CAMERA_PIPELINE = 0;
    private static final int LEFT_CAMERA_PORT_TO_FORWARD = 5802;
    private static final String LEFT_CAMERA_CONFIG_FILE = "CameraConfigs/Camera2/config.json";
    private static final int LEFT_CAMERA_RESOLUTION_WIDTH = 640;
    private static final int LEFT_CAMERA_RESOLUTION_HEIGHT = 480;
    private static final double LEFT_CAMERA_POSITION_X = Units.inchesToMeters(10.664); //9.664);
    private static final double LEFT_CAMERA_POSITION_Y = Units.inchesToMeters(11.5); //10.583);
    private static final double LEFT_CAMERA_POSITION_Z = Units.inchesToMeters(18); //15.891);
    private static final double LEFT_CAMERA_ROTATION_ROLL = Units.degreesToRadians(0.0);
    private static final double LEFT_CAMERA_ROTATION_PITCH = Units.degreesToRadians(0.0);
    private static final double LEFT_CAMERA_ROTATION_YAW = Units.degreesToRadians(33.0);

    static final Transform3d robotToLeftCam = 
        new Transform3d(
            new Translation3d(LEFT_CAMERA_POSITION_X, LEFT_CAMERA_POSITION_Y, LEFT_CAMERA_POSITION_Z),
            new Rotation3d(LEFT_CAMERA_ROTATION_ROLL,LEFT_CAMERA_ROTATION_PITCH, LEFT_CAMERA_ROTATION_YAW));

    // Right camera constants
    private static final String RIGHT_CAMERA_NAME = "Camera4"; 
    private static final String RIGHT_CAMERA_IP_NAME = "10.9.30.34";
    private static final int RIGHT_CAMERA_PIPELINE = 0;
    private static final int RIGHT_CAMERA_PORT_TO_FORWARD = 5804;
    private static final String RIGHT_CAMERA_CONFIG_FILE = "CameraConfigs/Camera4/config.json";
    private static final int RIGHT_CAMERA_RESOLUTION_WIDTH = 640;
    private static final int RIGHT_CAMERA_RESOLUTION_HEIGHT = 480;
    private static final double RIGHT_CAMERA_POSITION_X = Units.inchesToMeters(10.664); //9.664);
    private static final double RIGHT_CAMERA_POSITION_Y = Units.inchesToMeters(-11.5); //-10.583);
    private static final double RIGHT_CAMERA_POSITION_Z = Units.inchesToMeters(18); //15.891);
    private static final double RIGHT_CAMERA_ROTATION_ROLL = Units.degreesToRadians(0.0);
    private static final double RIGHT_CAMERA_ROTATION_PITCH = Units.degreesToRadians(0.0);
    private static final double RIGHT_CAMERA_ROTATION_YAW = Units.degreesToRadians(-18.0);

    static final Transform3d robotToRightCam = 
    new Transform3d(
        new Translation3d(RIGHT_CAMERA_POSITION_X, RIGHT_CAMERA_POSITION_Y, RIGHT_CAMERA_POSITION_Z),
        new Rotation3d(RIGHT_CAMERA_ROTATION_ROLL, RIGHT_CAMERA_ROTATION_PITCH, RIGHT_CAMERA_ROTATION_YAW));

    // Three cameras on the robot, 2 in the front, 1 on the back
    //private final CameraOnRobot m_backCamera;
    //private final CameraOnRobot m_rightCamera;
    private final CameraOnRobot m_leftCamera;

    private SwerveDriveKinematics m_swerveDriveKinematics;
    private Rotation2d m_rotation;
    private SwerveModulePosition[] m_swerveModulePositions;
    private Pose2d m_position;
    private SwerveDrivePoseEstimator m_PoseEstimator;
    
    // New Photon Pose Estimator Objects
    // -- These replaced old utility methods used to utilize multi tags
    private PhotonPoseEstimator photonPoseEstimator_FrontLeft;
    private PhotonPoseEstimator photonPoseEstimator_FrontRight;
    //private PhotonPoseEstimator photonPoseEstimator_Back;

    // Confidence level, 0 means that we have 100% confidence in the odometry position and it won't use camera values
    private Matrix<N3, N1> m_StateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    
    // Confidence level, 0 means that we have 100% confidence in the camera values and it won't trust odometry positions
    private Matrix<N3, N1> m_VisionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));//0.4 0.45

    private AprilTagFieldLayout tagLayout;

    /**
     * <h3>OdometryUtility</h3>
     * 
     * This contstructs an april tag version of the photonvision
     * 
     * @param swerveDriveKinematics class used to convert the velocity of the robot chassis to module states like speed and angle
     * @param rotation used to represent a rotation of the robot for the pose estimator
     * @param swerveModulePositions used to represent the states of the swerve modules
     * @param position used to represent a Pose2D of the robot for the pose estimator
     */
    public OdometryUtility(SwerveDriveKinematics swerveDriveKinematics, Rotation2d rotation, 
                           SwerveModulePosition[] swerveModulePositions, Pose2d position) {

        m_swerveDriveKinematics = swerveDriveKinematics;
        m_rotation = rotation;
        m_swerveModulePositions = swerveModulePositions;
        m_position = position;

       

        m_PoseEstimator = new SwerveDrivePoseEstimator(
            m_swerveDriveKinematics,
            m_rotation,
            m_swerveModulePositions,
            m_position,
            m_StateStdDevs,
            m_VisionMeasurementStdDevs
        );

        // Creates the cameras
        // m_backCamera = new CameraOnRobot(BACK_CAMERA_NAME, 
        //                                 BACK_CAMERA_IP_NAME, 
        //                                 BACK_CAMERA_PIPELINE, 
        //                                 BACK_CAMERA_PORT_TO_FORWARD,
        //                                 BACK_CAMERA_CONFIG_FILE,
        //                                 BACK_CAMERA_RESOLUTION_WIDTH,
        //                                 BACK_CAMERA_RESOLUTION_HEIGHT,
        //                                 BACK_CAMERA_POSITION_X,
        //                                 BACK_CAMERA_POSITION_Y,
        //                                 BACK_CAMERA_POSITION_Z,
        //                                 BACK_CAMERA_ROTATION_ROLL,
        //                                 BACK_CAMERA_ROTATION_PITCH,
        //                                 BACK_CAMERA_ROTATION_YAW
        //                                 );
        m_leftCamera = new CameraOnRobot(LEFT_CAMERA_NAME, 
                                        LEFT_CAMERA_IP_NAME, 
                                        LEFT_CAMERA_PIPELINE, 
                                        LEFT_CAMERA_PORT_TO_FORWARD,
                                        LEFT_CAMERA_CONFIG_FILE,
                                        LEFT_CAMERA_RESOLUTION_WIDTH,
                                        LEFT_CAMERA_RESOLUTION_HEIGHT,
                                        LEFT_CAMERA_POSITION_X,
                                        LEFT_CAMERA_POSITION_Y,
                                        LEFT_CAMERA_POSITION_Z,
                                        LEFT_CAMERA_ROTATION_ROLL,
                                        LEFT_CAMERA_ROTATION_PITCH,
                                        LEFT_CAMERA_ROTATION_YAW
                                        );
        // m_rightCamera = new CameraOnRobot(RIGHT_CAMERA_NAME, 
        //                                 RIGHT_CAMERA_IP_NAME, 
        //                                 RIGHT_CAMERA_PIPELINE, 
        //                                 RIGHT_CAMERA_PORT_TO_FORWARD,
        //                                 RIGHT_CAMERA_CONFIG_FILE,
        //                                 RIGHT_CAMERA_RESOLUTION_WIDTH,
        //                                 RIGHT_CAMERA_RESOLUTION_HEIGHT,
        //                                 RIGHT_CAMERA_POSITION_X,
        //                                 RIGHT_CAMERA_POSITION_Y,
        //                                 RIGHT_CAMERA_POSITION_Z,
        //                                 RIGHT_CAMERA_ROTATION_ROLL,
        //                                 RIGHT_CAMERA_ROTATION_PITCH,
        //                                 RIGHT_CAMERA_ROTATION_YAW
        //                                 );

       try{
        // Read tag Json
        tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        // Adjusts AprilTag position based on 0,0 based on alliance selection
        setOriginBasedOnAlliance();
        // Initialize Photon Pose Estimators
        photonPoseEstimator_FrontLeft = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, m_leftCamera.getPhotonCamera(), robotToLeftCam);
        //photonPoseEstimator_FrontLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //photonPoseEstimator_FrontRight = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, m_rightCamera.getPhotonCamera(), robotToRightCam);
        //photonPoseEstimator_FrontRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        //photonPoseEstimator_Back = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP, m_backCamera.getPhotonCamera(), robotToBackCam);
        //photonPoseEstimator_Back.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        } catch (IOException e) {
            DriverStation.reportWarning("Unable to load ChargedUp AprilTag resource file" + 
                                        AprilTagFields.k2023ChargedUp.m_resourceFile, e.getStackTrace());
        }
    }

    /**
     * <h3>setOriginBasedOnAlliance</h3>
     * 
     * in our odometry it places the robot on a starting spot of the alliance color you are on.
     */
    public void setOriginBasedOnAlliance() {

        for(int i=1; i<9;i++) {
            Optional<Pose3d> pose = tagLayout.getTagPose(i);
            if(pose != null && !pose.isEmpty() )
            {
              SmartDashboard.putNumberArray(this.getClass().getSimpleName()+"/AprilTags/Before/"+i, LogUtil.toPoseArray3d(pose.get()));
            }
        }

        AprilTagFieldLayout.OriginPosition originPos;
        if(DriverStation.getAlliance() == Alliance.Blue) {
            originPos = AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
            Logger.getInstance().recordOutput("OdometryUtility/alliance", "Blue");
        } else {
            originPos = AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
            Logger.getInstance().recordOutput("OdometryUtility/alliance", "Red");

        }
        tagLayout.setOrigin(originPos);

        for(int i=1; i<9;i++) {
          Optional<Pose3d> pose = tagLayout.getTagPose(i);
          if(pose != null && !pose.isEmpty() )
          {
            SmartDashboard.putNumberArray(this.getClass().getSimpleName()+"/AprilTags/After/"+i, LogUtil.toPoseArray3d(pose.get()));
          }
        }
    }

    /**
     * <h3>getBackCamera</h3>
     * 
     * Returns the PhotonCamera representing the back camera
     * 
     * @return a reference to the back camera
     */
    // public PhotonCamera getBackCamera() { 
    //     return m_backCamera.getPhotonCamera();
    // }

    /**
     * <h3>getLeftCamera</h3>
     * 
     * Returns the PhotonCamera representing the left camera
     * 
     * @return a reference to the left camera
     */
    public PhotonCamera getLeftCamera() { 
        return m_leftCamera.getPhotonCamera();
    }

    /**
     * <h3>getRightCamera</h3>
     * 
     * Returns the PhotonCamera representing the right camera
     * 
     * @return a reference to the right camera
     */
    // public PhotonCamera getRightCamera() { 
    //      return m_rightCamera.getPhotonCamera();
    //  }

    /**
     * <h3>updateCameraPos</h3>
     * 
     * Updates the position of the robot based on the april tag position
     * 
     * @param rotation2d Rotation of the camera in relation to the April Tag
     * @param swerveModulePositions Positions of the swerve modules
     * @param pose2d X and Y in relation to the April Tag
     */
    public void updateCameraPos(Rotation2d rotation2d, SwerveModulePosition[] swerveModulePositions, Pose2d pose2d) {
        m_PoseEstimator.update(rotation2d, swerveModulePositions);
        updateCameraPositions();
    }
    

    /**
     * <h3>getPose</h3>
     * 
     * Returns the estimated position of the robot based on the PoseEstimator
     * 
     * @return a reference to the position
     */
    public Pose2d getPose() {
        return m_PoseEstimator.getEstimatedPosition();
    }

    /**
     * <h3> updateCameraPositions </h3>
     * 
     * Updates the swerve estimated pose based on the position camera detections of April Tags
     */
    public void updateCameraPositions() {
        //SmartDashboard.putNumberArray(this.getClass().getSimpleName()+"/RobotPose", LogUtil.toPoseArray2d(getPose()));
        //TODO: may want to change other values sent to shuffleboard to be set to logger
       Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/RobotPose", getPose());
        try{
            // only runs cameras if we are in auto
            final Optional<EstimatedRobotPose> updatedEstimatedPose_FromLeft = photonPoseEstimator_FrontLeft.update();
            //final Optional<EstimatedRobotPose> updatedEstimatedPose_FromRight = photonPoseEstimator_FrontRight.update();
            if (!updatedEstimatedPose_FromLeft.isEmpty() && updatedEstimatedPose_FromLeft.isPresent()) {
                Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/CameraLeft", updatedEstimatedPose_FromLeft.get().estimatedPose.toPose2d());
            }
            // if (!updatedEstimatedPose_FromRight.isEmpty() &&updatedEstimatedPose_FromRight.isPresent()) {
            //     Logger.getInstance().recordOutput(this.getClass().getSimpleName()+"/CameraRight", updatedEstimatedPose_FromRight.get().estimatedPose.toPose2d());
            // }
           // final Optional<EstimatedRobotPose> updatedEstimatedPose_FromBack = photonPoseEstimator_Back.update();
            if (false) {//!DriverStation.isTeleop()){
                // final Optional<EstimatedRobotPose> updatedEstimatedPose_FromLeft = photonPoseEstimator_FrontLeft.update();
                // final Optional<EstimatedRobotPose> updatedEstimatedPose_FromRight = photonPoseEstimator_FrontRight.update();
                // final Optional<EstimatedRobotPose> updatedEstimatedPose_FromBack = photonPoseEstimator_Back.update();
                boolean seeLeftRobot = false;
                boolean seeRightRobot = false;
                boolean seeBackRobot = false;
                
                if (!updatedEstimatedPose_FromLeft.isEmpty() && updatedEstimatedPose_FromLeft.isPresent()) {
                    EstimatedRobotPose camPose = updatedEstimatedPose_FromLeft.get();
                    if(camPose.targetsUsed.size() > 1) {
                        m_PoseEstimator.addVisionMeasurement(
                                camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
                        seeLeftRobot = true;
                    }
                }
                SmartDashboard.putString("OdometryUtility/SeeLeftRobot", seeLeftRobot?"true":"false");

                // if (!updatedEstimatedPose_FromRight.isEmpty() &&updatedEstimatedPose_FromRight.isPresent()) {
                //     EstimatedRobotPose camPose = updatedEstimatedPose_FromRight.get();
                //     if(camPose.targetsUsed.size() > 1) {
                //         m_PoseEstimator.addVisionMeasurement(
                //             camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
                //         seeRightRobot = true;
                //     }
                // }
                // SmartDashboard.putString("OdometryUtility/SeeRightRobot", seeRightRobot?"true":"false");

                // if (!updatedEstimatedPose_FromBack.isEmpty() &&updatedEstimatedPose_FromBack.isPresent()) {
                //     EstimatedRobotPose camPose = updatedEstimatedPose_FromBack.get();
                //     if(camPose.targetsUsed.size() > 1) {
                //         m_PoseEstimator.addVisionMeasurement(
                //             camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
                //         seeBackRobot = true;
                //     }
                // }
                //SmartDashboard.putString("OdometryUtility/SeeBackRobot", seeBackRobot?"true":"false");
            }
                
            // SmartDashboard.putNumberArray(this.getClass().getSimpleName()+"/AdjustedRobotPose", LogUtil.toPoseArray2d(pose.toPose2d()));
            // SmartDashboard.putNumberArray(this.getClass().getSimpleName()+"/updatedEstimatedPose", LogUtil.toPoseArray3d(updatedEstimatedPose_FromLeft.get().estimatedPose));
        }
        catch(Exception e){
            DriverStation.reportError(" JS Error", e.getStackTrace());
            SmartDashboard.putString("OdometryUtility/e", e.getMessage());
        };
        // for(int i = 0; i < this.cameras.size(); i++) {
        //     final PhotonCamera camera = this.cameras.get(i).getPhotonCamera();
        //     final Transform3d offset = this.cameraOffsets.get(i);
        //     final PhotonPipelineResult result = camera.getLatestResult();
        //    if (!result.hasTargets()) continue;
        // }
    }

    /**
     * <h3>resetPosition</h3>
     * 
     * resets the position in the pose estimator
     * 
     * @param yaw a rotation of the robot
     * @param modulePositions the positions of the swerve modules
     * @param initialPose the initial pse2d of the robot
     */
    public void resetPosition(Rotation2d yaw, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        m_PoseEstimator.resetPosition(yaw, modulePositions, initialPose);
    }
}
