package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmIOSim implements ArmIO {
    private final SingleJointedArmSim sim = 
        new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            75, 
            0.58, 
            Units.inchesToMeters(45), 
            -Math.PI/4, 
            2*Math.PI, 
            Units.lbsToKilograms(11), 
            true
        );

    @Override
    public ArmInputs updateInputs() {
        if (DriverStation.isDisabled()) {
            this.sim.setInputVoltage(0.0);
        }
        sim.update(0.02);
        return new ArmInputs(MathUtil.clamp(
            sim.getOutput(0), -12, 12),
            Units.radiansToDegrees(sim.getAngleRads()),
            Units.radiansToDegrees(sim.getVelocityRadPerSec()));
    }

    @Override
    public void setVoltage(double volts) {
        SmartDashboard.putNumber("Arm Controller Effort", volts);
        sim.setInputVoltage(volts);
    }
    
}
