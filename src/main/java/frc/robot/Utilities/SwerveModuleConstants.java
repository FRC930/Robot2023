package frc.robot.utilities;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SwerveModuleConstants {

    
  
    public final int driveEncoderChannel;
    public final int turningEncoderChannel;
    public final  boolean turningEncoderReversed;
    public final boolean driveEncoderReversed;
    public final int cancoderID;
    public final double angleOffset;
 


    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(int driveMotorChannel,
    int turningMotorChannel,
    int driveEncoderChannel,
    int turningEncoderChannel,
    int cancoderID,
    boolean driveEncoderReversed,
    boolean turningEncoderReversed,
    double angleOffset) {
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
        this.driveEncoderChannel = driveEncoderChannel;
        this.turningEncoderChannel = turningEncoderChannel;
        this.driveEncoderReversed = driveEncoderReversed;
        this.turningEncoderReversed = turningEncoderReversed;
        
    };
}
