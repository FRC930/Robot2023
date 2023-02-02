package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

public class ArmIOSparks implements ArmIO {

    private CANSparkMax armMotor; 
    private RelativeEncoder armEncoder;

    public ArmIOSparks(int WristMotorID, int ShoulderMotorID) {
        armMotor = new CANSparkMax(ShoulderMotorID, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();

        armMotor.setSmartCurrentLimit(20);

        armEncoder = armMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        armEncoder.setPositionConversionFactor(360);
        armEncoder.setVelocityConversionFactor(60);
    }

    @Override
    public ArmInputs updateInputs() {
        return new ArmInputs(
            MathUtil.clamp(armMotor.getOutputCurrent(), -12, 12),
            armEncoder.getPosition(),
            armEncoder.getVelocity()
        );
        
    }

    @Override
    public void setVoltage(double volts) {
        armMotor.setVoltage(volts);
    }
    
}
