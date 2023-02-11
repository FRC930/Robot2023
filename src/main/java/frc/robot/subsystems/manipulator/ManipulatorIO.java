package frc.robot.subsystems.manipulator;

public interface ManipulatorIO {
    
    public void updateInputs();
    public double getRollerVoltage();
    public double getCurrentAngleDegrees();
    public double getVelocityDegreesPerSecond();
    public void setVoltage(double volts);
    public void setRollerSpeed(double speed);
}