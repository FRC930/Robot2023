package frc.robot.utilities;

// ----- IMPORTS ----- \\

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

/**
 * <h3>AprilVisionUtility</h3>
 * 
 * AprilVisionUtility holds the cameras that we use for our aiming.
 */
public class AprilVisionUtility {

    // ----- CONSTANTS ----- \\
    // The pipeline that you want to select
    private static final int PI_CAMERA_INDEX = 0;

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
    private final Transform2d m_CameraPosition = new Transform2d(new Translation2d(Units.inchesToMeters(6), 0.0), new Rotation2d(3.14159));

    // ----- CONSTRUCTOR ----- \\
    /**
     * <h3>AprilVisionUtility</h3>
     * 
     * This contstructs an april tag version of the photonvision
     */
    public AprilVisionUtility(SwerveDriveKinematics swerveDriveKinematics, Rotation2d rotation, SwerveModulePosition[] swerveModulePositions, Pose2d position) {

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

        // Creates the camera
        m_PhotonCamera = new PhotonCamera("PiCamera");
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
        PhotonPipelineResult vResult = m_PhotonCamera.getLatestResult();
//TODO 2D Should we bw using Pose 2D Also?
        m_PoseEstimator.update(rotation2d, swerveModulePositions);
        if(vResult.hasTargets()) {
            PhotonTrackedTarget vTarget = vResult.getBestTarget();
            int vTargetID = vTarget.getFiducialId();

            if(CameraTargetUtility.getInstance().targetExists(vTargetID)) {

                Translation2d vTargetTranslation = vTarget.getBestCameraToTarget().getTranslation().toTranslation2d();
                Rotation2d vTargetRotation = vTarget.getBestCameraToTarget().getRotation().toRotation2d();

                Transform2d vTargetTransform = new Transform2d(vTargetTranslation, vTargetRotation);

                Pose2d vInvertCamPose = CameraTargetUtility.getInstance().getTarget(vTargetID).getTargetPos().transformBy(vTargetTransform.inverse());
                Pose2d vVisionMeasurment = vInvertCamPose.transformBy(m_CameraPosition);
                double vImageCaptureTime = Timer.getFPGATimestamp() - (vResult.getLatencyMillis() / 1000d); 
                m_PoseEstimator.addVisionMeasurement(vVisionMeasurment, vImageCaptureTime);

                if (Robot.isReal()) {
                    try {
                        m_PoseEstimator.updateWithTime(Timer.getFPGATimestamp(), m_rotation, m_swerveModulePositions);
                    } catch (Exception e) {
                        // TODO: Handle exception
                    }
                }

            }
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
}
