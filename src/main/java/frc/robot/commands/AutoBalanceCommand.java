package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/**
     * <h3>AutoBalanceCommand</h3>
     * 
     * Balances the robot on the Charging Station
     * 
     */
public class AutoBalanceCommand extends CommandBase{
    
    private final double Pitch_Deadband = 5.0;

    private SwerveDrive m_swerveDrive;

    private Rotation2d robotPitch;
    
    private double robotPitchInDegrees;

    private double throttle;
    private double strafe;
    private double rotation;
    private boolean isFieldRelative;
    private boolean isOpenLoop;

    /**
     * <h3>AutoBalanceCommand</h3>
     * 
     * Balances the robot on the Charging Station
     * 
     * @param swerveDrive Drive subsystem
     */
    public AutoBalanceCommand(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
        robotPitchInDegrees = 0.0;
        throttle = 0.0;
        strafe = 0.0;
        rotation = 0.0;
        isFieldRelative = true;
        isOpenLoop = true;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        robotPitch = m_swerveDrive.getPitch();
        robotPitchInDegrees = robotPitch.getDegrees();
        Logger.getInstance().recordOutput("AutoBalanceCommand/robotPitch", robotPitchInDegrees);

        double tempSpeed = 0.0;
        if (robotPitchInDegrees/100 <= -0.12) {
            tempSpeed = -0.5;
        } 
        else if (robotPitchInDegrees/100 >= 0.12) {
            tempSpeed = 0.5;
        }
        else {
            tempSpeed = robotPitchInDegrees/100;
        }

        throttle = -tempSpeed;
        
        m_swerveDrive.drive(throttle, strafe, rotation, isFieldRelative, isOpenLoop);
    }

    @Override
    public boolean isFinished() {
        if (robotPitchInDegrees < Pitch_Deadband && robotPitchInDegrees > - Pitch_Deadband) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
