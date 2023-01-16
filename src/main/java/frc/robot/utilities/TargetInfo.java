package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * <h3>TargetInfo</h3>
 * 
 * TargetInfo Holds information about the April Tags
 */
public class TargetInfo {
    
    // ----- VARIABLES ----- \\
    Pose2d m_TagPosition;

    // ----- CONSTRUCTOR ----- \\
    /**
     * <h3>TargetInfo</h3>
     * 
     * This constructs the target info of an april tag
     */
    public TargetInfo(Pose2d TagPosition){
        m_TagPosition = TagPosition;
    }

    // ------ METHODS ------ \\
    /**
     * <h3>getTargetPos</h3>
     * 
     * Returns an april tag's position on the field
     * s
     * @return a reference to april tag positition
     */
    public Pose2d getTargetPos() {
        return m_TagPosition;
    }
}
