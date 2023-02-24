package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utilities.SparkMaxWrapper;

public class ManipulatorIOSim implements ManipulatorIO {

    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 75,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(9.4), Units.lbsToKilograms(4)),
            Units.inchesToMeters(9.4), -2 * Math.PI, 2 * Math.PI, true);
    private final SparkMaxWrapper roller = new SparkMaxWrapper(15, MotorType.kBrushless);

      /**
     * <h3>updateInputs</h3>
     */
    @Override
    public void updateInputs() {
        sim.update(0.02);
    }
    
    /**
     * <h3>getCurrentAngleDegrees</h3>
     * Gets the wrist motor position in degrees.
     * 
     * @return the wrist motor position.
     */
    @Override
    public double getCurrentAngleDegrees() {
        return Units.radiansToDegrees(sim.getAngleRads());
    }

    /**
     * <h3>getVelocityDegreesPerSecond</h3>
     * Gets wrist motor velocity in degrees per second, converted from radians to
     * degree
     * 
     * @return The wrist motor velocity
     */
    @Override
    public double getVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(sim.getVelocityRadPerSec());
    }

    /**
     * <h3>setVoltage
     * <h3>
     * Sets the wrist motor voltage
     * @param volts
     */
    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }
    /**
     * <h3>getRollaerVoltagee
     * <h3>
     * gets the roller voltage
     * 
     * @return getBusVoltage
     */
    @Override
    public double getRollerVoltage() {
        return roller.getBusVoltage();
    }
    /**
     * <h3>setRollerSpeed
     * <h3>
     * sets the speed for the roller
     * 
     * @param speed
     */
    @Override
    public void setRollerSpeed(double speed) {
        roller.set(speed);
    }
}
