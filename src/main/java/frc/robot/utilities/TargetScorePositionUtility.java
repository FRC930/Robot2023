package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TargetScorePositionUtility {
    public enum Target{
        high,
        medium,
        low
    };

    private Target m_desiredTarget;

    public TargetScorePositionUtility() {
        m_desiredTarget = Target.low;
    }

    public void setDesiredTarget(Target target) {
        m_desiredTarget = target;
    }

    public Target getDesiredTarget() {
        return m_desiredTarget;
    }

    public boolean isHigh() {
        return m_desiredTarget == Target.high ;
    }

    public boolean isMedium() {
        return m_desiredTarget == Target.medium ;
    }

    public boolean isLow() {
        return m_desiredTarget == Target.low ;
    } 


    public Command setDesiredTargetCommand(Target desiredTarget) {
        return new InstantCommand(() -> setDesiredTarget(desiredTarget));
    }
}
