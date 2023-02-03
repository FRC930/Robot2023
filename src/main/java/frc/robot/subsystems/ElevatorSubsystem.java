package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    // The CANSparkMax motor (Left Motor) follows the Right motor at the same time
    private final CANSparkMax RightElevatorMaster;
    //private final CANSparkMax LeftElevatorFollower;
    // The Endoder is going to track what the distance of the motor is 
    private final RelativeEncoder rightElevatorEncoder;
    private final ProfiledPIDController controller = new ProfiledPIDController(1, 0, 0, new Constraints(360, 360));
   
    public ElevatorSubsystem(int rightMotorID){//int leftMotorID, int rightMotorID){

        //LeftElevatorFollower = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        RightElevatorMaster = new CANSparkMax(rightMotorID, MotorType.kBrushless);

        rightElevatorEncoder = RightElevatorMaster.getEncoder();
        // reset motors 
    
        //LeftElevatorFollower.restoreFactoryDefaults();
        RightElevatorMaster.restoreFactoryDefaults();

        //LeftElevatorFollower.follow(RightElevatorMaster, true);
    }
    /**
     * <h2>getElevatorPosition</h2>
     * @return
     */
    public double getElevatorPosition() {
        return rightElevatorEncoder.getPosition();
    }

    public void setElevatorVoltage(double targetPosition) {
        RightElevatorMaster.setVoltage(controller.calculate(rightElevatorEncoder.getPosition(), targetPosition));
    }
    //CHANGE ELEVATOR POSITION FROM THIS METHOD INSTEAD OF COMMAND AND CALL THIS METHOD IN COMMAND TO SIMPLIFY COMMAND
    public void setElevatorPosition(){

    }

    public void stopMotors() {
        RightElevatorMaster.stopMotor();
    }
}
