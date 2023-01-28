package frc.robot.utilities;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

// ----- IMPORTS ----- \\

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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
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

    // ----- CONSTANTS ----- \\
    // The pipeline that you want to select
    private static final int PI_CAMERA_INDEX = 0;

    // TODO robot to camera
    private static final double CAMERA_POSITION_X = 10.0;
    private static final double CAMERA_POSITION_Y = 10.0;
    private static final double CAMERA_POSITION_Z = 25.0;

    // ----- VARIABLES ----- \\
    // This is the camera that is used for april tags
    private PhotonCamera m_PhotonCamera;

    private SwerveDriveKinematics m_swerveDriveKinematics;
    private Rotation2d m_rotation;
    private SwerveModulePosition[] m_swerveModulePositions;
    private Pose2d m_position;
    private SwerveDrivePoseEstimator m_PoseEstimator;
    private final Matrix<N3, N1> m_StateStdDevs = VecBuilder.fill(0.0, 0.0, Units.degreesToRadians(0));
    private final Matrix<N3, N1> m_VisionMeasurementStdDevs = VecBuilder.fill(0.0, 0.0, Units.degreesToRadians(0));

    private AprilTagFieldLayout tagLayout;

    private List<PhotonCamera> cameras;

    // ----- CONSTRUCTOR ----- \\
    /**
     * <h3>OdometryUtility</h3>
     * 
     * This contstructs an april tag version of the photonvision
     */
    public OdometryUtility(SwerveDriveKinematics swerveDriveKinematics, Rotation2d rotation, SwerveModulePosition[] swerveModulePositions, Pose2d position) {

        m_swerveDriveKinematics = swerveDriveKinematics;
        m_rotation = rotation;
        m_swerveModulePositions = swerveModulePositions;
        m_position = position;
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        m_PoseEstimator = new SwerveDrivePoseEstimator(
            m_swerveDriveKinematics,
            m_rotation,
            m_swerveModulePositions,
            m_position,
            m_StateStdDevs,
            m_VisionMeasurementStdDevs
        );

        // Creates the camera
        m_PhotonCamera = new PhotonCamera("PiCamera");
        //TODO
        // camera1 = new PhotonCamera(instance, "front-left");
        // camera2 = new PhotonCamera(instance, "front-right");
        // camera3 = new PhotonCamera(instance, "back");
        cameras = List.of(m_PhotonCamera);
        m_PhotonCamera.setPipelineIndex(PI_CAMERA_INDEX);
    }
    

    // ------ METHODS ------ \\

    /**
     * <h3>getHubTrackingCamera</h3>
     * 
     * Returns the PhotonCamera representing the hub aiming camera
     * s
     * @return a reference to the hub camera
     */
    public PhotonCamera getHubTrackingCamera() {
        return m_PhotonCamera;
    }

    /**
     * <h3>updateCameraPos</h3>
     * 
     * Updates the position of the robot based on the april tag position
     * @param pose2d
     * @param swerveModulePositions
     * @param rotation2d
     * 
     */
    public void updateCameraPos(Rotation2d rotation2d, SwerveModulePosition[] swerveModulePositions, Pose2d pose2d) {
        m_PoseEstimator.update(rotation2d, swerveModulePositions);
        updateCameraPositions();
    }
    
   /**
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
        if(knownTags == null || corners == null ||
                corners.size() != knownTags.size()*4 || knownTags.size() == 0) {
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
     *  Updates the swerve estimated pose based on the position camera detections of April Tags
     */
    public void updateCameraPositions() {
        SmartDashboard.putNumberArray("RobotPose", LogUtil.toPoseArray2d(getPose()));
    
        final List<TargetCorner> corners = new ArrayList<TargetCorner>();
        final List<AprilTag> foundTags = new ArrayList<AprilTag>();
    
        for(int i = 0; i < this.cameras.size(); i++) {
            final PhotonCamera camera = this.cameras.get(i);
            final PhotonPipelineResult result = camera.getLatestResult();
            if (!result.hasTargets()) continue;
    
            corners.clear();
            foundTags.clear();
    
            final List<PhotonTrackedTarget> targets = result.getTargets();
    
            targets.stream().forEach( target -> {
                final int fiducialId = target.getFiducialId();
    
                corners.addAll(target.getDetectedCorners());
                foundTags.add(new AprilTag(
                    fiducialId,
                    this.tagLayout.getTagPose(fiducialId).get()
                ));
            });
    
            if (targets.size() > 1) {
    
                CameraProperties cameraProp;
                PNPResults pnpResults = null;
                try {
                    cameraProp = new CameraProperties("CameraConfigs/Camera1/config.json", 640, 480);
                    pnpResults = OdometryUtility.estimateCamPosePNP(cameraProp, corners, foundTags);
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } 
                final Transform3d robotToCameraPose = new Transform3d(
                new Translation3d(
                    Units.inchesToMeters(CAMERA_POSITION_X),
                    Units.inchesToMeters(CAMERA_POSITION_Y),
                    Units.inchesToMeters(CAMERA_POSITION_Z)
                ),
                new Rotation3d(
                    0,
                    -Math.toRadians(18),
                    Math.toRadians(30)
                )
            );
    
                SmartDashboard.putNumber(camera.getName() + "/multi/ambiguity", pnpResults.ambiguity);
                SmartDashboard.putNumber(camera.getName() + "/multi/bestErr", pnpResults.bestReprojErr);
                SmartDashboard.putNumber(camera.getName() + "/multi/altErr", pnpResults.altReprojErr);
    
                if(pnpResults != null && pnpResults.bestReprojErr < 0.15) {
                    final Pose3d pose = new Pose3d()
                        .plus(pnpResults.best)
                        .plus(robotToCameraPose.inverse());
                    
                    addVisionMeasurement(pose.toPose2d(), result.getLatencyMillis() / 1000.0);
                }
            }
        }
    }

    /**
     * <h3>addVisionMeasurement</h3>
     * 
     * Updates the swerve pose estimator based on the vision latency 
     * @param measurement the estimated Pose2d
     * @param latencySeconds time the camera lags
     */
    private void addVisionMeasurement(Pose2d measurement, double latencySeconds) {
        m_PoseEstimator.addVisionMeasurement(
            measurement,
            Timer.getFPGATimestamp() - latencySeconds
        );
    }
}
