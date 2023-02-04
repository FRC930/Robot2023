package frc.robot.utilities;

public class DesiredPitchUtility {
    private static DesiredPitchUtility instance;
    private double desiredPosition;

    private DesiredPitchUtility() {
        desiredPosition = 0.0;
    }

    /**
     * <h3>getInstance</h3>
     * 
     * ShuffleboardUtility is a singleton, so getInstance returns the instance of
     * the class that the program will use
     * 
     * @return the instance
     */
    public static DesiredPitchUtility getInstance() {
        if (instance == null) {
            instance = new DesiredPitchUtility();
        }
        return instance;
    }

    public void setDesiredPosition(double position) {
        desiredPosition = position;
    }

    public double getDesiredPosition() {
        return desiredPosition;
    }
}
