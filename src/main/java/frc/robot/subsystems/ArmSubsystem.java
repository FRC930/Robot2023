package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

private CANSparkMax wristMotor;
private CANSparkMax armMotor; 

private RelativeEncoder wristEncoder;
private RelativeEncoder armEncoder;

//TODO: These are nonsensical (Fix once we get actual values)
public static double highWristPosition = 27.6; //at high elevator position
public static double highArmPosition = 10; //at high elevator position

public static double mediumWristPosition = 23.9; //at medium elevator position
public static double mediumArmPosition = 17.9; //at medium elevator position

public static double groundWristPosition = 5.2; //at ground elevator position
public static double groundArmPosition = 46.8; //at ground elevator position

public static double stowWristPosition = 90.0; //at ground elevator position
public static double stowArmPosition = -60.0; //at ground elevator position

public static double intakeWristPosition = -225.0;
public static double intakeArmPosition = 90.0;

    public ArmSubsystem (int WristMotorID, int ShoulderMotorID) {
        wristMotor = new CANSparkMax(WristMotorID, MotorType.kBrushless);
        armMotor = new CANSparkMax(ShoulderMotorID, MotorType.kBrushless);

        wristMotor.restoreFactoryDefaults();
        armMotor.restoreFactoryDefaults(); 

        wristEncoder = wristMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);
        armEncoder = armMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        wristEncoder.setPositionConversionFactor(360);
        armEncoder.setPositionConversionFactor(360);
    }

    public void setWristSpeed(double speed) {
        wristMotor.set(speed);
    }

    public void setArmSpeed(double speed) {
        armMotor.set(speed);
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public void stopWristMotor() {
        wristMotor.stopMotor();
    }

    public void stopArmMotor() {
        armMotor.stopMotor();
    }
}
