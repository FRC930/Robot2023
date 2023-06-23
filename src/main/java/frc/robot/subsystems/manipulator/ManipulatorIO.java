package frc.robot.subsystems.manipulator;

// this defines what functions each mechanical arm must contain
    // these functions are primarily used during simulation
public interface ManipulatorIO {
    
    public void updateInputs();
    public double getRollerVoltage();
    public double getCurrentAngleDegrees();
    public double getVelocityDegreesPerSecond();
    public void setVoltage(double volts);
    public void setRollerSpeed(double speed);
    public double getRollerCurrent();
    public double getRealCurrentAngleDegrees();
}