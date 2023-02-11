package frc.robot.subsystems.arm;

public interface ArmIO {
    public void updateInputs();
    public double getCurrentAngleDegrees();
    public double getVelocityDegreesPerSecond();
    public void setVoltage(double volts);
}
