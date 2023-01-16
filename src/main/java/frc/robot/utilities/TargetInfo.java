package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;

public class TargetInfo {
    
    Pose2d m_TagPosition;

    public TargetInfo(Pose2d TagPosition){
        m_TagPosition = TagPosition;
    }

    public Pose2d getTargetPos() {
        return m_TagPosition;
    }
}
