package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(27.12), Units.lbsToKilograms(11)), Units.inchesToMeters(27.12), 0, 2 * Math.PI, Units.lbsToKilograms(11), true);
    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(DCMotor.getNEO(1), 75, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(27.12), Units.lbsToKilograms(11)), Units.inchesToMeters(27.12), 0, 2 * Math.PI, Units.lbsToKilograms(11), true);

    @Override
    public void updateInputs() {
        armSim.update(0.02);
        wristSim.update(0.02);
    }

    @Override
    public double getWristOutputVoltage() {
        return MathUtil.clamp(wristSim.getOutput(0), -12, 12);
    }

    @Override
    public double getWristCurrentAngleDegrees() {
        return Units.radiansToDegrees(wristSim.getAngleRads());
    }

    @Override
    public double getWristVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(wristSim.getVelocityRadPerSec());
    }

    @Override
    public void setWristVoltage(double volts) {
        wristSim.setInputVoltage(volts);
    }

    @Override
    public double getShoulderOutputVoltage() {
        return MathUtil.clamp(armSim.getOutput(0), -12, 12);
    }

    @Override
    public double getShoulderCurrentAngleDegrees() {
        return Units.radiansToDegrees(armSim.getAngleRads());
    }

    @Override
    public double getShoulderVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    }

    @Override
    public void setShoulderVoltage(double volts) {
        armSim.setInputVoltage(volts);
    }
}
