package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            4.0,
            Units.lbsToKilograms(9.8),
            Units.inchesToMeters(1.756),
            Units.inchesToMeters(0),
            Units.inchesToMeters(52),
            true
        );

    /**
     * <h3>updateInputs</h3>
     * 
     * updates the inputs for the motor sim
     */
    @Override
    public void updateInputs() {
        sim.update(0.02);
    }

    /**
     * <h3>getOutputVoltage</h3>
     * 
     * Gets the elevator motor outputs in volts
     * @return the elevator motor voltage
     */
    @Override
    public double getCurrentVelocity() {
        return sim.getVelocityMetersPerSecond();
    }

    /**
     * <h3>getCurrentHeight</h3>
     * 
     * Gets the elevator motor position in meters
     * @return the elevator motor position
     */
    @Override
    public double getCurrentHeight() {
        return sim.getPositionMeters();
    }

    /**
     * <h3>setVoltage</h3>
     * 
     * Set the elevator motor voltage 
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }
}
