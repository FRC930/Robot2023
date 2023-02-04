package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

    private final ElevatorSim sim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            4.0,
            Units.lbsToKilograms(9.8),
            Units.inchesToMeters(1),
            Units.inchesToMeters(0),
            Units.inchesToMeters(22.64),
            true
        );

    @Override
    public void updateInputs() {
        sim.update(0.02);
    }

    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(sim.getOutput(0), -12, 12);
    }

    @Override
    public double getCurrentVelocity() {
        return Units.metersToInches(sim.getVelocityMetersPerSecond());
    }

    @Override
    public double getCurrentHeight() {
        return Units.metersToInches(sim.getPositionMeters());
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }
}
