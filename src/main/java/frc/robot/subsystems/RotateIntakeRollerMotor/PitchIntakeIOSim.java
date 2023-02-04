package frc.robot.subsystems.RotateIntakeRollerMotor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PitchIntakeIOSim implements IntakeMotorIO{
    
    private final SingleJointedArmSim sim = 
        new SingleJointedArmSim(DCMotor.getNeo550(1), 6, SingleJointedArmSim.estimateMOI(Units.inchesToMeters(3), Units.lbsToKilograms(3)), Units.inchesToMeters(3), 0, Math.PI/2, 0, false);

    @Override
    public void updateInputs() {
        sim.update(0.02);
    }

    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(sim.getOutput(0), -12, 12);
    }

    @Override
    public double getCurrentAngleDegrees() {
        return Units.radiansToDegrees(sim.getAngleRads());
    }

    @Override
    public double getCurrentVelocityDegreesPerSecond() {
        return Units.radiansToDegrees(sim.getVelocityRadPerSec());
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);        
    }
}

