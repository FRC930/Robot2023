package frc.robot.subsystems.manipulator;

// TODO document interface
public interface ManipulatorIO {
    
    public void updateInputs();
    public double getRollerVoltage();
    public double getCurrentAngleDegrees();
    public double getVelocityDegreesPerSecond();
    public void setVoltage(double volts);
    public void setRollerSpeed(double speed);
}