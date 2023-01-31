package frc.robot.utilities;

import java.io.File;
import java.io.FileReader;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * <h3>CameraTargetUtility</h3>
 * 
 * CameraTargetUtility Holds information about targets
 */
public class CameraTargetUtility {

    private static CameraTargetUtility vInstance = null;

    // ----- VARIABLES ----- \\
    private Map<Integer, TargetInfo> m_Targets;
    private static JSONParser m_parser; 
    private JSONArray m_Taglist;
    private Iterator<JSONObject > m_Iterator;

    // ----- CONSTRUCTOR ----- \\
    /**
     * <h3>CameraTargetUtility</h3>
     * 
     * This constructs the april tag manager and should only ever be called once by the
     * getInstance method
     */
    private CameraTargetUtility() {

        m_Targets = new HashMap<Integer, TargetInfo>() ;
        m_parser = new JSONParser();
        try{
            //Creates a JSON object based on the april tag JSON file.
            JSONObject vobj = (JSONObject) m_parser.parse(new FileReader(new File(Filesystem.getDeployDirectory(), "AprilTagInfo/2023-chargedup.json")));
            
            m_Taglist = (JSONArray) vobj.get("tags");
            m_Iterator = m_Taglist.iterator();
            
            //Gets information from the april tag JSON file and updates the target info to match the tag the robot is currently looking at
            while (m_Iterator.hasNext()){

                JSONObject vThisObj = (JSONObject) m_Iterator.next();
                int vThisTagID = Integer.parseInt(vThisObj.get("ID").toString());
                JSONObject vThisPose = (JSONObject) vThisObj.get("pose");

                JSONObject vThisTrans = (JSONObject) vThisPose.get("translation");
                double vThisXPos = Double.parseDouble(vThisTrans.get("x").toString());
                double vThisYPos = Double.parseDouble(vThisTrans.get("y").toString());
                double vThisZPos = Double.parseDouble(vThisTrans.get("z").toString());

                JSONObject vThisRot = (JSONObject) vThisPose.get("rotation");
                JSONObject vThisQuat = (JSONObject) vThisRot.get("quaternion");
                double vThisWRot = Double.parseDouble(vThisQuat.get("W").toString());
                double vThisXRot = Double.parseDouble(vThisQuat.get("X").toString());
                double vThisYRot = Double.parseDouble(vThisQuat.get("Y").toString());
                double vThisZRot = Double.parseDouble(vThisQuat.get("Z").toString());

                Quaternion tagPosition = new Quaternion(vThisWRot, vThisXRot, vThisYRot, vThisZRot);
                TargetInfo vThisInfo = new TargetInfo(new Pose3d(vThisXPos, vThisYPos, vThisZPos, new Rotation3d(tagPosition)).toPose2d());
                addTarget(vThisTagID, vThisInfo);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
        // ----- SINGLETON GET ----- \\

        /**
         * <h3>getInstance</h3>
         * 
         * This is the accessor method for the singleton. This ensures that there is
         * only ever one instance of the CameraTargetUtility
         * 
         * @return the instance of CameraTargetUtility
         */
        public static CameraTargetUtility getInstance() {
            if (vInstance == null) {
                vInstance = new CameraTargetUtility();
            }
            return vInstance;
        }

        // ------ METHODS ------ \\

        /**
         * <h3>getTarget</h3>
         * 
         * Returns TargetInfo representing the info of an april tag
         * s
         * @return a reference to april tag info
         */
        public TargetInfo getTarget(Integer vID) {
            return m_Targets.get(vID);
        }

        /**
         * <h3>addTarget</h3>
         * 
         * Adds a target to the map of targets
         * 
         */
        public void addTarget(Integer vID, TargetInfo vInfo) {
            m_Targets.put(vID, vInfo);
        }

        /**
         * <h3>targetExists</h3>
         * 
         * Returns whether or not a target exists
         * s
         * @return whether a target exists
         */
        public boolean targetExists(Integer vID) {
            return (null != m_Targets.get(vID));
        }
}