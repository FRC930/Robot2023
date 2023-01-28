package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

private CANSparkMax elbowMotor;
private CANSparkMax shoulderMotor; 

//TODO: These are nonsensical (Fix once we get actual values)
public static double highElbowPosition = -45.0;
public static double highShoulderPosition = 150.0;

public static double mediumElbowPosition = -45.0;
public static double mediumShoulderPosition = 120.0;

public static double groundElbowPosition = -50.0;
public static double groundShoulderPosition = 90.0;

public static double stowElbowPosition = 0.0;
public static double stowShoulderPosition = 0.0;

public static double intakeElbowPosition = -225.0;
public static double intakeShoulderPosition = 90.0;

    public ArmSubsystem (int ElbowID, int ShoulderID) {
        elbowMotor = new CANSparkMax(ElbowID, MotorType.kBrushless);
        shoulderMotor = new CANSparkMax(ShoulderID, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        shoulderMotor.restoreFactoryDefaults(); 

        // TODO: Put in new API, Alternate Encoders through BORE Encoder
        elbowMotor.getEncoder().setPositionConversionFactor(360);
        shoulderMotor.getEncoder().setPositionConversionFactor(360);

    }

    public void setElbowSpeed(double speed) {
        elbowMotor.set(speed);
    }

    public void setShoulderSpeed(double speed) {
        shoulderMotor.set(speed);
    }

    public double getElbowPosition() {
        return elbowMotor.getEncoder().getPosition();
    }

    public double getShoulderPosition() {
        return shoulderMotor.getEncoder().getPosition();
    }

    public void stopElbowMotor() {
        elbowMotor.stopMotor();
    }

    public void stopShoulderMotor() {
        shoulderMotor.stopMotor();
    }
}
