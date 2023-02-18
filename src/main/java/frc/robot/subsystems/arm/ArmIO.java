package frc.robot.subsystems.arm;

/**
 * <h3>ArmIO</h3>
 * 
 * Sets up the methods that we use in the IORobot and IOSim
 */
public interface ArmIO {
    public void updateInputs();
    public double getCurrentAngleDegrees();
    public double getVelocityDegreesPerSecond();
    public void setVoltage(double volts);
}
