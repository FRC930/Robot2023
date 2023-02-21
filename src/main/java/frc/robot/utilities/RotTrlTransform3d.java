package frc.robot.utilities;

import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Represents a transformation that first rotates a pose around the origin,
 * and then translates it.
 */
public class RotTrlTransform3d {
    private final Translation3d trl;
    private final Rotation3d rot;

    public RotTrlTransform3d() {
        this(new Rotation3d(), new Translation3d());
    }

    /**
     * <h3>rotTrlTransform3d</h3>
     * 
     * Creates a rotation-translation transformation from a Transform3d.
     * 
     * <p>
     * Applying this transformation to poses will preserve their current
     * origin-to-pose
     * transform as if the origin was transformed by these components.
     * 
     * @param trf The origin transformation
     */
    public RotTrlTransform3d(Transform3d trf) {
        this(trf.getRotation(), trf.getTranslation());
    }

    /**
     * <h3>rotTrlTransform3d<h3>
     * A rotation-translation transformation.
     * 
     * <p>
     * Applying this transformation to poses will preserve their current
     * origin-to-pose
     * transform as if the origin was transformed by these components.
     * 
     * @param rot The rotation component
     * @param trl The translation component
     */
    public RotTrlTransform3d(Rotation3d rot, Translation3d trl) {
        this.rot = rot;
        this.trl = trl;
    }

    /**
     * <h3>makeRelativeTo</h3>
     * The rotation-translation transformation that makes poses in the world
     * consider this pose as the new origin, or change the basis to this pose.
     * 
     * @param pose The new origin
     */
    public static RotTrlTransform3d makeRelativeTo(Pose3d pose) {
        return new RotTrlTransform3d(pose.getRotation(), pose.getTranslation()).inverse();
    }

    /**
     * <h3>inverse</h3>
     * The inverse of this transformation. Applying the inverse will "undo" this
     * transformation.
     */
    public RotTrlTransform3d inverse() {
        var inverseRot = rot.unaryMinus();
        var inverseTrl = trl.rotateBy(inverseRot).unaryMinus();
        return new RotTrlTransform3d(inverseRot, inverseTrl);
    }

    /** 
     * <h3>getTransform3d</h3>
     * 
     * This transformation as a Transform3d (as if of the origin) 
     * @return transform3d
     */
    public Transform3d getTransform() {
        return new Transform3d(trl, rot);
    }

    /** 
     * <h3>getTranslation</h3>
     * The translation component of this transformation
     * @return trl
      */
    public Translation3d getTranslation() {
        return trl;
    }

    /** 
     * <h3>getRotation</h3>
     * The rotation component of this transformation 
     * @return rot
     */
    public Rotation3d getRotation() {
        return rot;
    }

    /**
     * <h3>apply</h3>
     * returns  the apply pose3D and new rotation
     * @param trl
     * @return apply 
     */
    public Translation3d apply(Translation3d trl) {
        return apply(new Pose3d(trl, new Rotation3d())).getTranslation();
    };

    /**
     * <h3>applyTrls</h3>
     * returns the applied trls
     * @param trls
     * @return trls
     */
    public List<Translation3d> applyTrls(List<Translation3d> trls) {
        return trls.stream().map(t -> apply(t)).collect(Collectors.toList());
    }

    /**
     * <h3>apply</h3>
     * returns the pose3d
     * @param pose
     * @return pose3d
     */
    public Pose3d apply(Pose3d pose) {
        return new Pose3d(
                pose.getTranslation().rotateBy(rot).plus(trl),
                pose.getRotation().plus(rot));
    }

    /**
     * <h3>applyposes</h3>
     * returns the poses
     * @param poses
     * @return poses.stream
     */
    public List<Pose3d> applyPoses(List<Pose3d> poses) {
        return poses.stream().map(p -> apply(p)).collect(Collectors.toList());
    }
}
