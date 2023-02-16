package frc.robot.subsystems.rotateintake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * 
 * <h3>PitchIntakeIOSim</h3>
 * 
 * Simulates the PitchIntakeMotor
 * 
 */
public class PitchIntakeIOSim implements IntakeMotorIO{
    
    // TODO get gearing, jKgMetersSquared, weight,
    private final SingleJointedArmSim sim = 
        new SingleJointedArmSim(DCMotor.getNeo550(1), 67, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(5.125), Units.lbsToKilograms(3)), Units.inchesToMeters(5.125), -180, 180, false);

        @Override
    public void updateInputs() {
        sim.update(0.02);
    }

    /**
     * <h3>getOutputVoltage</h3>
     * 
     * Returns the motors voltage between -12 and 12
     * 
     */
    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(sim.getOutput(0), -12, 12);
    }

    /**
     * <h3>getCurrentAngleDegrees</h3>
     * 
     * Gets the current position of the motor in radians and converts them into degrees
     * 
     */
    @Override
    public double getCurrentAngleDegrees() {
        return Units.radiansToDegrees(sim.getAngleRads());
    }

    /**
     * <h3>getCurrentVelocityDegreesPerSecond</h3>
     * 
     * Gets current velocity from the motor in degrees per second
     * 
     */
    @Override
    public double getCurrentVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(sim.getVelocityRadPerSec());
    }

    /**
     * <h3>setVoltage</h3>
     * 
     * Sets the imput voltage value in the sim
     * 
     */
    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);        
    }
}

