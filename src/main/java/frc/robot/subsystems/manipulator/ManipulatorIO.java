package frc.robot.subsystems.manipulator;

public interface ManipulatorIO {
    
    public void updateInputs();
    public double getOutputVoltage();
    public double getCurrentAngleDegrees();
    public double getVelocityDegreesPerSecond();
    public void setVoltage(double volts);
}