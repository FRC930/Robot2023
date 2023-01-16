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
 * CameraTargetUtility Holds information about the April Tags
 */
public class CameraTargetUtility {

    private static CameraTargetUtility vInstance = null;

    // ----- VARIABLES ----- \\
    private Map<Integer, TargetInfo> m_Targets;
    private static JSONParser m_parser; 
    private JSONArray m_Taglist ;
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
            JSONObject vobj = (JSONObject) m_parser.parse(new FileReader(new File(Filesystem.getDeployDirectory(), "AprilTagInfo/2023-chargedup.json")));
            m_Taglist = (JSONArray) vobj.get("tags");
            m_Iterator = m_Taglist.iterator();
        
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
            System.out.println();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

        public static CameraTargetUtility getInstance() {
            if (vInstance == null) {
                vInstance = new CameraTargetUtility();
            }
            return vInstance;
        }

        public TargetInfo getTarget(Integer vID) {
            return m_Targets.get(vID);
        }

        public void addTarget(Integer vID, TargetInfo vInfo) {
            m_Targets.put(vID, vInfo);
        }

        public boolean targetExists(Integer vID) {
            return (null != m_Targets.get(vID));
        }
}