package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class Rotate extends CommandBase{

    private SwerveDrive m_swerveDrive;
    private Pose2d m_targetPose2d;
    private Pose2d currentPose2d;

    public Rotate(Pose2d targetPose, SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
        m_targetPose2d = targetPose;
    }
    //d=√((x2 – x1)² + (y2 – y1)²)
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
}
