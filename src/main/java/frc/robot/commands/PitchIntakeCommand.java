package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utilities.DesiredPitchUtility;

/**
 * <h3>PitchIntakeCommand</h3>
 * 
 * Moves the PitchIntakeMotor
 * 
 */
public class PitchIntakeCommand extends CommandBase{
    private double desiredPosition;

    /**
     * 
     * <h3>PitchIntakeCommand</h3>
     * 
     * Moves the PitchIntakeMotor
     * 
     * @param pitchIntakeSubsystem The PitchIntakeMotor
     * @param voltage The desired voltage for the motor
    */
    public PitchIntakeCommand(double desiredPosition) {
        this.desiredPosition = desiredPosition;
    }

    @Override
    public void initialize() {
        DesiredPitchUtility.getInstance().setDesiredPosition(desiredPosition);
    }


    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}
