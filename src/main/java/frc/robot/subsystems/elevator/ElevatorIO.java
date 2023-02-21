package frc.robot.subsystems.elevator;

/**
 * <h3>ElevatorIO</h3>
 * 
 * Sets up the methods that we use in the IORobot and IOSim
 */
public interface ElevatorIO {
    public void updateInputs();
    public double getCurrentVelocity();
    public double getCurrentHeight();
    public void setVoltage(double volts);
}
