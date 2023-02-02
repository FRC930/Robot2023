package frc.robot.subsystems.arm;

public interface ArmIO {
    public record ArmInputs(
        double outputVoltage,
        double currentAngleDegrees,
        double currentVelocityDegreesPerSecond
    ) {}

    public ArmInputs updateInputs();
    public void setVoltage(double volts);
}