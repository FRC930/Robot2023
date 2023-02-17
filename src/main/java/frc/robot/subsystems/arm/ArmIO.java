package frc.robot.subsystems.arm;

// TODO document interface
public interface ArmIO {
    public void updateInputs();
    public double getCurrentAngleDegrees();
    public double getVelocityDegreesPerSecond();
    public void setVoltage(double volts);
}
