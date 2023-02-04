package frc.robot.subsystems.RotateIntakeRollerMotor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;

public class PitchIntakeIORobot implements IntakeMotorIO {

    private CANSparkMax m_RotateIntakeRollerMotor;
    private RelativeEncoder m_RotateIntakeRollerEncoder;
    
    public PitchIntakeIORobot(int motorID, int encoderID) {
        //Creates the motor
        m_RotateIntakeRollerMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        m_RotateIntakeRollerMotor.restoreFactoryDefaults();

        //Creates the encoder
        m_RotateIntakeRollerEncoder = m_RotateIntakeRollerMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 42);
        m_RotateIntakeRollerEncoder.setPositionConversionFactor(360);
        //Figure out what number the factor has to be
        m_RotateIntakeRollerEncoder.setVelocityConversionFactor(60);
    }

    @Override
    public double getOutputVoltage() {
        return MathUtil.clamp(m_RotateIntakeRollerMotor.getOutputCurrent(), -12, 12);
    }

    @Override
    public double getCurrentAngleDegrees() {
        return m_RotateIntakeRollerEncoder.getPosition();
    }

    @Override
    public double getCurrentVelocityDegreesPerSecond() {
        return m_RotateIntakeRollerEncoder.getVelocity();
    }

    @Override
    public void setVoltage(double volts) {
        m_RotateIntakeRollerMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs() {}
    
}
