package frc.robot.subsystems.elevator;

public interface ElevatorIO {
    public void updateInputs();
    public double getOutputVoltage();
    public double getCurrentVelocity();
    public double getCurrentHeight();
    public void setVoltage(double volts);
}
