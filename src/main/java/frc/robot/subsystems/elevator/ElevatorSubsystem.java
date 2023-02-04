package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    
    private final ElevatorIO io;
    private double targetElevatorPosition;

    private final ProfiledPIDController controller;
    private final ElevatorFeedforward ff;
   
    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
        this.controller = new ProfiledPIDController(1, 0, 0, new Constraints(360, 360));
        this.ff = new ElevatorFeedforward(0, 0, 0, 0);
    }

    public void setTargetElevatorPosition(double inches){
        targetElevatorPosition = inches;
    }

    @Override
    public void periodic() {
        io.updateInputs();

        double voltage = controller.calculate(io.getCurrentHeight(), targetElevatorPosition);
        double feedforward = ff.calculate(io.getCurrentVelocity());
        MathUtil.clamp(voltage, -12, 12);

        io.setVoltage(voltage + feedforward);
    }
}
