package frc.robot.utilities;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.utilities.vision.estimation.CameraProperties;
import frc.robot.utilities.vision.estimation.OpenCVHelp;
import frc.robot.utilities.vision.estimation.PNPResults;
import frc.robot.utilities.vision.estimation.TargetModel;


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
    // private static final String BACK_CAMERA_NAME = "Camera3"; 
    // private static final String BACK_CAMERA_IP_NAME = "10.9.30.33";
    // private static final int BACK_CAMERA_PIPELINE = 0;
    // private static final int BACK_CAMERA_PORT_TO_FORWARD = 5803;
    // private static final String BACK_CAMERA_CONFIG_FILE = "CameraConfigs/Camera3/config.json";
    // private static final int BACK_CAMERA_RESOLUTION_WIDTH = 640;
    // private static final int BACK_CAMERA_RESOLUTION_HEIGHT = 480;
    // private static final double BACK_CAMERA_POSITION_X = Units.inchesToMeters(10.0);
    // private static final double BACK_CAMERA_POSITION_Y = Units.inchesToMeters(10.0);
    // private static final double BACK_CAMERA_POSITION_Z = Units.inchesToMeters(25.0);
    // private static final double BACK_CAMERA_ROTATION_ROLL = Math.toRadians(0.0);;
    // private static final double BACK_CAMERA_ROTATION_PITCH = Math.toRadians(0.0);;
    // private static final double BACK_CAMERA_ROTATION_YAW = Math.toRadians(0.0);

    //Left camera constants
    private static final String LEFT_CAMERA_NAME = "Camera1"; 
    private static final String LEFT_CAMERA_IP_NAME = "10.9.30.31`";
    private static final int LEFT_CAMERA_PIPELINE = 0;
    private static final int LEFT_CAMERA_PORT_TO_FORWARD = 5801;
    private static final String LEFT_CAMERA_CONFIG_FILE = "CameraConfigs/Camera1/config.json";
    private static final int LEFT_CAMERA_RESOLUTION_WIDTH = 320;
    private static final int LEFT_CAMERA_RESOLUTION_HEIGHT = 240;
    private static final double LEFT_CAMERA_POSITION_X = Units.inchesToMeters(9.0);
    private static final double LEFT_CAMERA_POSITION_Y = Units.inchesToMeters(12.5);
    private static final double LEFT_CAMERA_POSITION_Z = Units.inchesToMeters(18.5);
    private static final double LEFT_CAMERA_ROTATION_ROLL = Units.degreesToRadians(0.0);
    private static final double LEFT_CAMERA_ROTATION_PITCH = Units.degreesToRadians(0.0);
    private static final double LEFT_CAMERA_ROTATION_YAW = Units.degreesToRadians(30.0);

    // Right camera constants
    private static final String RIGHT_CAMERA_NAME = "Camera2"; 
    private static final String RIGHT_CAMERA_IP_NAME = "10.9.30.32";
    private static final int RIGHT_CAMERA_PIPELINE = 0;
    private static final int RIGHT_CAMERA_PORT_TO_FORWARD = 5802;
    private static final String RIGHT_CAMERA_CONFIG_FILE = "CameraConfigs/Camera2/config.json";
    private static final int RIGHT_CAMERA_RESOLUTION_WIDTH = 320;
    private static final int RIGHT_CAMERA_RESOLUTION_HEIGHT = 240;
    private static final double RIGHT_CAMERA_POSITION_X = Units.inchesToMeters(9.0);
    private static final double RIGHT_CAMERA_POSITION_Y = -Units.inchesToMeters(11.0);
    private static final double RIGHT_CAMERA_POSITION_Z = Units.inchesToMeters(18.5);
    private static final double RIGHT_CAMERA_ROTATION_ROLL = Units.degreesToRadians(0.0);;
    private static final double RIGHT_CAMERA_ROTATION_PITCH = Units.degreesToRadians(0.0);;
    private static final double RIGHT_CAMERA_ROTATION_YAW = Units.degreesToRadians(-30.0);;

    // Three cameras on the robot, 2 in the front, 1 on the back
    //private final CameraOnRobot m_backCamera;
    private final CameraOnRobot m_rightCamera;
    private final CameraOnRobot m_leftCamera;

    private SwerveDriveKinematics m_swerveDriveKinematics;
    private Rotation2d m_rotation;
    private SwerveModulePosition[] m_swerveModulePositions;
    private Pose2d m_position;
    private SwerveDrivePoseEstimator m_PoseEstimator;
    // Confidence level, 0 means that we have 100% confidence in the odometry position and it won't use camera values
    //TODO Bumped up to 0.5 for testing, please put back down
    private Matrix<N3, N1> m_StateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    // Confidence level, 0 means that we have 100% confidence in the camera values and it won't trust odometry positions
    private Matrix<N3, N1> m_VisionMeasurementStdDevs = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(90));

    private AprilTagFieldLayout tagLayout;

    private List<CameraOnRobot> cameras;

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

        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            DriverStation.reportWarning("Unable to load ChargedUp AprilTag resource file" + 
                                        AprilTagFields.k2023ChargedUp.m_resourceFile, e.getStackTrace());
        }

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
        m_rightCamera = new CameraOnRobot(RIGHT_CAMERA_NAME, 
                                        RIGHT_CAMERA_IP_NAME, 
                                        RIGHT_CAMERA_PIPELINE, 
                                        RIGHT_CAMERA_PORT_TO_FORWARD,
                                        RIGHT_CAMERA_CONFIG_FILE,
                                        RIGHT_CAMERA_RESOLUTION_WIDTH,
                                        RIGHT_CAMERA_RESOLUTION_HEIGHT,
                                        RIGHT_CAMERA_POSITION_X,
                                        RIGHT_CAMERA_POSITION_Y,
                                        RIGHT_CAMERA_POSITION_Z,
                                        RIGHT_CAMERA_ROTATION_ROLL,
                                        RIGHT_CAMERA_ROTATION_PITCH,
                                        RIGHT_CAMERA_ROTATION_YAW
                                        );

        cameras = List.of();//m_rightCamera, m_leftCamera); //m_backCamera );

       // Adjusts AprilTag position based on 0,0 based on alliance selection
       setOriginBasedOnAlliance();
    }

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
        } else {
            originPos = AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;
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
    //     return m_rightCamera.getPhotonCamera();
    // }

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
    * <h3>estimateCamPosePNP</h3>

     * Performs solvePNP using 3d-2d point correspondences to estimate the field-to-camera transformation.
     * If only one tag is visible, the result may have an alternate solution.
     * 
     * <p><b>Note:</b> The returned transformation is from the field origin to the camera pose!
     * 
     * @param prop The camera properties
     * @param corners The visible tag corners in the 2d image
     * @param knownTags The known tag field poses corresponding to the visible tag IDs
     * @return The transformation that maps the field origin to the camera pose
     */
    public static PNPResults estimateCamPosePNP(
        CameraProperties prop, List<TargetCorner> corners, List<AprilTag> knownTags) {
            if(knownTags == null || corners == null || corners.size() != knownTags.size()*4 || knownTags.size() == 0) {
                return new PNPResults();
            }

            // single-tag pnp
            if(corners.size() == 4) {
                var camToTag = OpenCVHelp.solvePNP_SQUARE(prop, TargetModel.kTag16h5.vertices, corners);
                var bestPose = knownTags.get(0).pose.transformBy(camToTag.best.inverse());
                var altPose = new Pose3d();
                if(camToTag.ambiguity != 0) altPose = knownTags.get(0).pose.transformBy(camToTag.alt.inverse());
                var o = new Pose3d();
                return new PNPResults(
                    new Transform3d(o, bestPose),
                    new Transform3d(o, altPose),
                    camToTag.ambiguity, camToTag.bestReprojErr, camToTag.altReprojErr
                );
            }

            // multi-tag pnp
            else {
                var objectTrls = new ArrayList<Translation3d>();
                for(var tag : knownTags) objectTrls.addAll(TargetModel.kTag16h5.getFieldVertices(tag.pose));
                var camToOrigin = OpenCVHelp.solvePNP_SQPNP(prop, objectTrls, corners);
                // var camToOrigin = OpenCVHelp.solveTagsPNPRansac(prop, objectTrls, corners);
                return new PNPResults(
                    camToOrigin.best.inverse(),
                    camToOrigin.alt.inverse(),
                    camToOrigin.ambiguity, camToOrigin.bestReprojErr, camToOrigin.altReprojErr);
            }
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
        SmartDashboard.putNumberArray(this.getClass().getSimpleName()+"/RobotPose", LogUtil.toPoseArray2d(getPose()));
    
        final List<TargetCorner> corners = new ArrayList<TargetCorner>();
        final List<AprilTag> foundTags = new ArrayList<AprilTag>();
    
        for(int i = 0; i < this.cameras.size(); i++) {
            final PhotonCamera camera = this.cameras.get(i).getPhotonCamera();
            final PhotonPipelineResult result = camera.getLatestResult();
            if (!result.hasTargets()) continue;
    
            corners.clear();
            foundTags.clear();
    
            final List<PhotonTrackedTarget> targets = result.getTargets();
    
            targets.stream().forEach( target -> {
                final int fiducialId = target.getFiducialId();
  
                Optional<Pose3d> unknownTag = this.tagLayout.getTagPose(fiducialId); 

               // if(!unknownTag.isEmpty()){
                    corners.addAll(target.getDetectedCorners());
                    foundTags.add(new AprilTag(
                        fiducialId,
                        unknownTag.get()
                    ));
                //}
            });
            
            // TODO make 0 greater than 1
            if (targets.size() > 1) {

                SmartDashboard.putNumber(this.getClass().getSimpleName()+"/NumberOfCorners", corners.size());
                SmartDashboard.putNumber(this.getClass().getSimpleName()+"/NumberOfTags", foundTags.size());
    
                final Transform3d robotToCameraPose = cameras.get(i).getRobotToCameraPose();
                CameraProperties cameraProp = cameras.get(i).getCameraProp();
                PNPResults pnpResults = OdometryUtility.estimateCamPosePNP(cameraProp, corners, foundTags);;
                SmartDashboard.putNumber(this.getClass().getSimpleName()+"/BestRepojErr", pnpResults.bestReprojErr);

                SmartDashboard.putNumber(this.getClass().getSimpleName()+ "/Multi/Ambiguity", pnpResults.ambiguity);
                SmartDashboard.putNumber(this.getClass().getSimpleName()+ "/Multi/BestErr", pnpResults.bestReprojErr);
                SmartDashboard.putNumber(this.getClass().getSimpleName()+ "/Multi/AltErr", pnpResults.altReprojErr);

                if(pnpResults != null && (pnpResults.bestReprojErr < 0.15 && targets.size() > 1) || (pnpResults.ambiguity < 0.2 && targets.size() == 1)) {
                    final Pose3d pose = new Pose3d()
                        .plus(pnpResults.best)
                        .plus(robotToCameraPose.inverse());
                    
                    addVisionMeasurement(pose.toPose2d(), result.getLatencyMillis() / 1000.0);
                    SmartDashboard.putNumberArray(this.getClass().getSimpleName()+"/AdjustedRobotPose", LogUtil.toPoseArray2d(pose.toPose2d()));
                }
            }
        }
    }

    /**
     * <h3>addVisionMeasurement</h3>
     * 
     * Updates the swerve pose estimator based on the vision latency
     *  
     * @param measurement the estimated Pose2d
     * @param latencySeconds time the camera lags
     */
    private void addVisionMeasurement(Pose2d measurement, double latencySeconds) {
        m_PoseEstimator.addVisionMeasurement(
            measurement,
            Timer.getFPGATimestamp() - latencySeconds
        );
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
