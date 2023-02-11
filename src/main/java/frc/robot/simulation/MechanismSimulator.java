package frc.robot.simulation;

import org.ejml.data.DMatrixRMaj;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;
import frc.robot.subsystems.rotateintake.PitchIntakeSubsystem;

public class MechanismSimulator {
    private final Mechanism2d mech;
    private final MechanismRoot2d root;
    private final MechanismLigament2d m_elevator;
    private final MechanismLigament2d m_arm;
    private final MechanismLigament2d m_hand;
    private final MechanismLigament2d m_intake;

    private final ArmSubsystem arm;
    private final ManipulatorSubsystem manipulator;
    private final ElevatorSubsystem elevator;
    private final PitchIntakeSubsystem intake;

    private final SwerveDrive swerve;

    private static double elevatorYOffset = 0;
    private static double elevatorZOffset = 12.07;
    private static double elevatorXOffset = 0;

    DoubleArrayLogEntry elevatorPose3d;
    DoubleArrayLogEntry armPose3d;
    DoubleArrayLogEntry manipulatorPose3d;

    /**
     * Simulates the arm and elevator systems in simulation, in a 2d window.
     * 
     * @param arm Subsystem for the arm.
     * @param elevator Subsystem for the elevator.
     */
    public MechanismSimulator(ArmSubsystem arm, ElevatorSubsystem elevator, ManipulatorSubsystem manipulator, PitchIntakeSubsystem intake, SwerveDrive swerve){
        this.arm = arm;
        this.elevator = elevator;
        this.manipulator = manipulator;
        this.intake = intake;
        this.swerve = swerve;

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
                arm.getPosition(),
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
                    manipulator.getPosition(),
                    6,
                    new Color8Bit(Color.kRed)
                    )
            );
    
        m_intake =
            root.append(
                new MechanismLigament2d(
                    "Intake",
                    10,
                    0,
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
        m_arm.setAngle(arm.getPosition());
        m_hand.setAngle(manipulator.getPosition()-arm.getPosition());
        m_intake.setAngle(intake.getEncoderPosition());

        // Advantage scope
        double[] elevatorPosition = {
            elevatorXOffset + elevator.getElevatorPosition() + swerve.getPose().getX(),
            elevatorYOffset + swerve.getPose().getY(),
            elevatorZOffset + elevator.getElevatorPosition(),
            1,
            0,
            0,
            swerve.getHeadingDegrees()};
        Logger.getInstance().recordOutput("Elevator Position", elevatorPosition);
    }
}
