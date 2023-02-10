package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.IntakeRollerMotorSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;

public class MechanismSimulator {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d m_elevator;
    private final MechanismLigament2d m_arm;
    private final MechanismLigament2d m_hand;
    private final MechanismLigament2d m_intake;

    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final PitchIntakeSubsystem intake;

    /**
     * Simulates the arm and elevator systems in simulation, in a 2d window.
     * 
     * @param arm Subsystem for the arm.
     * @param elevator Subsystem for the elevator.
     */
    public MechanismSimulator(ArmSubsystem arm, ElevatorSubsystem elevator, PitchIntakeSubsystem intake){
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;

        mech = new Mechanism2d(50, 50);
        root = mech.getRoot("robot", 20, 20);
        

        
        // Adds elevator to the robot in simulation
        m_elevator =
            root.append(
                new MechanismLigament2d(
                    "Elevator",
                    10,
                    45,
                    10,
                    new Color8Bit(Color.kGreen)
                )
            );
        
        // Adds arm to the elevator in simulation
        m_arm =
            m_elevator.append(
              new MechanismLigament2d(
                "Arm",
                10,
                arm.getShoulderPosition(),
                6,
                new Color8Bit(Color.kYellow)
                )  
            );
        
        // Adds manipulator to the arm in simulation
        m_hand =
            m_arm.append(
                new MechanismLigament2d(
                    "Hand",
                    5,
                    arm.getWristPosition(),
                    6,
                    new Color8Bit(Color.kRed)
                    )
            );
    
        m_intake =
            root.append(
                new MechanismLigament2d(
                    "Intake",
                    10,
                    this.intake.getEncoderPosition(),
                    6,
                    new Color8Bit(Color.kBlueViolet)
                )
            );
        
        // Sends system simulations to the smart dashboard
        SmartDashboard.putData("Mech2d", mech);
    }
    
    /**
     * Constantly updates current simulation angle to the display
     */
    public void periodic(){
        m_elevator.setLength(elevator.getElevatorPosition());
        m_arm.setAngle(arm.getShoulderPosition());
        m_hand.setAngle(arm.getWristPosition()-arm.getShoulderPosition());
        m_intake.setAngle(intake.getEncoderPosition());
    }
}
