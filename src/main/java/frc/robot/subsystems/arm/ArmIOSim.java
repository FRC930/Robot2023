package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 75, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(27.12), Units.lbsToKilograms(11)), Units.inchesToMeters(27.12), 0, 2 * Math.PI, Units.lbsToKilograms(11), true);

    /**
     * updates the inputs for the motor sim
     */
    @Override
    public void updateInputs() {
        sim.update(0.02);
    }

    /**
     * Gets the shoulder motor position in degrees
     * @return the shoulder motor position
     */
    @Override
    public double getCurrentAngleDegrees() {
        return Units.radiansToDegrees(sim.getAngleRads());
    }

    /**
     * Gets the Shoulder 
     * @return
     */
    @Override
    public double getVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(sim.getVelocityRadPerSec());
    }

    /**
     * Set the shoulder motor voltage 
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }
}
