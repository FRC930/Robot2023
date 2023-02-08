package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utilities.SparkMaxWrapper;

public class ManipulatorIOSim implements ManipulatorIO{
    
    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 75, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(27.12), Units.lbsToKilograms(11)), Units.inchesToMeters(27.12), 0, 2 * Math.PI, Units.lbsToKilograms(11), true);
    private final SparkMaxWrapper roller = new SparkMaxWrapper(15, MotorType.kBrushless);

    /**
     * updates the inputs for the motor sim
     */
    @Override
    public void updateInputs() {
        sim.update(0.02);
    }
    
    /**
     * Gets the wrist motor position in degrees.
     * @return the wrist motor position.
     */
    @Override
    public double getCurrentAngleDegrees() {
        return Units.radiansToDegrees(sim.getAngleRads());
    }

    /**
     * Gets wrist motor velocity in degrees per second.
     * @return The wrist motor velocity 
     */
    @Override
    public double getVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(sim.getVelocityRadPerSec());
    }

    /**
     * Sets the wrist motor voltage
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }

    @Override
    public double getRollerVoltage() {
        return roller.getBusVoltage();
    }

    @Override
    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }
}
