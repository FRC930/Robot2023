package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class RotatePositions {
    public static Pose2d zero_zero = new Pose2d(0, 0, new Rotation2d());

    private SendableChooser<Pose2d> m_chooser = new SendableChooser<>();

    public RotatePositions(){
        m_chooser.addOption("Zero_Zero", zero_zero);
    }

    public Pose2d getPose2dForRotation() {
        return m_chooser.getSelected();
    }
}
