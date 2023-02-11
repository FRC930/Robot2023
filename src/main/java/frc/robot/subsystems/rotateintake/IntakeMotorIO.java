package frc.robot.subsystems.rotateintake;

public interface IntakeMotorIO {
    public void updateInputs();
    public double getOutputVoltage();
    public double getCurrentAngleDegrees();
    public double getCurrentVelocityDegreesPerSecond();
    public void setVoltage(double volts);
}

