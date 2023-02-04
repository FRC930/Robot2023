package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(27.12), Units.lbsToKilograms(11)), Units.inchesToMeters(27.12), 0, 2 * Math.PI, Units.lbsToKilograms(11), true);
    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(27.12), Units.lbsToKilograms(11)), Units.inchesToMeters(27.12), 0, 2 * Math.PI, Units.lbsToKilograms(11), true);

    /**
     * updates the inputs for the motor sim
     */
    @Override
    public void updateInputs() {
        armSim.update(0.02);
        wristSim.update(0.02);
    }

    /**
     * Gets wrist motor outputs.
     * @return Clamped current Voltage of the wrist.
     */
    @Override
    public double getWristOutputVoltage() {
        return MathUtil.clamp(wristSim.getOutput(0), -12, 12);
    }
    /**
     * Gets the wrist motor position in degrees.
     * @return the wrist motor position.
     */
    @Override
    public double getWristCurrentAngleDegrees() {
        return Units.radiansToDegrees(wristSim.getAngleRads());
    }

    /**
     * Gets wrist motor velocity in degrees per second.
     * @return The wrist motor velocity 
     */
    @Override
    public double getWristVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(wristSim.getVelocityRadPerSec());
    }

    /**
     * Sets the wrist motor voltage
     * @param volts
     */
    @Override
    public void setWristVoltage(double volts) {
        wristSim.setInputVoltage(volts);
    }

    /**
     * Gets the shoulder motor outputs in volts
     * @return the sholder motor outputs 
     */
    @Override
    public double getShoulderOutputVoltage() {
        return MathUtil.clamp(armSim.getOutput(0), -12, 12);
    }

    /**
     * Gets the shoulder motor position in degrees
     * @return the shoulder motor position
     */
    @Override
    public double getShoulderCurrentAngleDegrees() {
        return Units.radiansToDegrees(armSim.getAngleRads());
    }

    /**
     * Gets the Shoulder 
     * @return
     */
    @Override
    public double getShoulderVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    }

    /**
     * Set the shoulder motor voltage 
     * @param volts
     */
    @Override
    public void setShoulderVoltage(double volts) {
        armSim.setInputVoltage(volts);
    }
}
