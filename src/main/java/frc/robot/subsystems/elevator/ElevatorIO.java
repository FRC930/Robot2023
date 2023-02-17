package frc.robot.subsystems.elevator;

// TODO document interface
public interface ElevatorIO {
    public void updateInputs();
    public double getCurrentVelocity();
    public double getCurrentHeight();
    public void setVoltage(double volts);
}
