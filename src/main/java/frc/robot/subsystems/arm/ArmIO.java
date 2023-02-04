package frc.robot.subsystems.arm;

public interface ArmIO {

    public void updateInputs();
    public double getWristOutputVoltage();
    public double getWristCurrentAngleDegrees();
    public double getWristVelocityDegreesPerSecond();
    public void setWristVoltage(double volts);
    public double getShoulderOutputVoltage();
    public double getShoulderCurrentAngleDegrees();
    public double getShoulderVelocityDegreesPerSecond();
    public void setShoulderVoltage(double volts);
}
