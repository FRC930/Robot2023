package frc.robot.subsystems.rotateintake;

// this defines what functions each mechanical arm must contain
    // these functions are primarily used during simulation
public interface IntakeMotorIO {
    public void updateInputs();
    public double getOutputVoltage();
    public double getCurrentAngleDegrees();
    public double getCurrentVelocityDegreesPerSecond();
    public void setVoltage(double volts);
}

