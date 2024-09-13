package frc.robot.subsystems.vision;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
public class CameraSystem{

    private final Map<Integer, Pose3d> fiducialMap = new HashMap<>();
    private ArrayList<PhotonCamera> cameras;
    private ArrayList<Transform3d> offsets;
    private ArrayList<PhotonPoseEstimator> estimators;
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    

    private static CameraSystem instance;

    private CameraSystem() {

        double inchesToMeters = 0.0254;
        cameras = new ArrayList<PhotonCamera>();
        offsets  = new ArrayList<Transform3d>();
        estimators = new ArrayList<PhotonPoseEstimator>();

        // Initialize fiducial map with Pose3d
        initializeFiducialMap(inchesToMeters);
    }
    // checks to see if the camera at the given position sees a tag
    public PhotonPipelineResult getResult(int position){
        return cameras.get(position).getLatestResult();
    }
    public PhotonCamera getCamera(int position)
    {
        return cameras.get(position);
    }
    // adds the camera, offset, and estimator to their arraylists; each camera, offset, and estimator have the same position in each arraylist
    public void AddCamera(PhotonCamera camera, Transform3d offset){
        cameras.add(camera);
        offsets.add(offset);
        estimators.add(new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, offset));
    }
    // calculates robot position
     public Pose2d calculateRobotPosition() {
        int cameraCount = 0;
        int cameraTagCount = 0;
        double sumX = 0;
        double sumY = 0;
        
        double rotationSumx = 0;
        double rotationSumY = 0;
        double rotationSumZ = 0;
        
        // Goes through every camera in the camera array list and checks if it sees a tag
        for(PhotonCamera cam : cameras)
        {
            if(cam.getLatestResult().hasTargets())
            {
                // if the camera picks up a tag, it calculates the position from the tag and runs it through a pose estimator
                Pose3d orig = calculatePoseFromCameraResult(cam.getLatestResult(), offsets.get(cameraCount));
                Optional<EstimatedRobotPose> estimatedPose = usePoseEstimator(cameraCount, orig.toPose2d());
                if(estimatedPose != null && !estimatedPose.isEmpty()){
                    Pose3d temp = estimatedPose.get().estimatedPose;
                    // add the components of the pose 3d to the sums
                sumX += temp.getX();
                sumY += temp.getY();
                rotationSumx += temp.getRotation().getX();
                rotationSumY += temp.getRotation().getY();
                rotationSumZ += temp.getRotation().getZ();              
                cameraTagCount++;     
                }
            }
            cameraCount++;
        
        }
        if(cameraTagCount > 0)
        {
        // Average X, Y, and Z from camera results
        double avgX = sumX / cameraTagCount;
        double avgY = sumY / cameraTagCount;
        // We can ignore Z for Pose2d
        Rotation3d avgRotation = new Rotation3d(
            (double)rotationSumx / cameraTagCount,
            (double)rotationSumY / cameraTagCount,
            (double)rotationSumZ/ cameraTagCount
        );

        // Convert the 3D Pose to 2D Pose
        return new Pose2d(avgX, avgY, new Rotation2d(avgRotation.getZ()));
        }
        // if no cameras detect tags, return blank Pose2d
        return new Pose2d();
        
    }
    // runs the orginal pose and puts it through it corresponding estimator
    private Optional<EstimatedRobotPose> usePoseEstimator(int position, Pose2d prevPose){
        estimators.get(position).setReferencePose(prevPose);
        return estimators.get(position).update();
    }
    // calculates the postition of tag to robot from one camera's results 
    private Pose3d calculatePoseFromCameraResult(PhotonPipelineResult result, Transform3d cameraOffset) {
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            
                
            // gets the position of the april tag scanned
            Pose3d fiducialPose = fiducialMap.get(target.getFiducialId());

            if (fiducialPose != null) {
                // calcuates the april tag position to the camera
                Transform3d transform = target.getBestCameraToTarget().inverse();
                Pose3d cameraToTargetPose = fiducialPose.transformBy(transform);
                // finds the pose of the robot from the camera position
                Pose3d robotPose3d = cameraToTargetPose.transformBy(cameraOffset);
                return new Pose3d(
                    robotPose3d.getX(),
                    robotPose3d.getY(),
                    robotPose3d.getZ(),
                    robotPose3d.getRotation()
                );
            }
        }
        return null;
    }
    public static CameraSystem getInstance() {
        if (instance == null) {
            instance = new CameraSystem();
        }
        return instance;
    }

    // checks to see if the result sees any april tags
    public boolean hasTargets(){
        int cameraCount = 0;
       for(PhotonCamera cam : cameras)
        {
            if(cam.getLatestResult().hasTargets()){
                return true;
            } 
            cameraCount++;
        }
        return false;
    }
    public double getTimeStamp() 
    {
        int cameraCount = 0;
        for (PhotonCamera cam : cameras){
            if(cam.getLatestResult().hasTargets()){
                Pose3d orig = calculatePoseFromCameraResult(cam.getLatestResult(), offsets.get(cameraCount));
                Optional<EstimatedRobotPose> estimatedPose = usePoseEstimator(cameraCount, orig.toPose2d());
                if(estimatedPose != null && !estimatedPose.isEmpty())
                    return estimatedPose.get().timestampSeconds;
            }
            cameraCount++;
        }
        return -1;
    }
    // returns a Double Object, will need to grab out the double value from the object and check if it is null
    public Double getYawForTag(int position){
            if(getResult(position).hasTargets() && (getResult(position).getBestTarget().getFiducialId() == 3
            || getResult(position).getBestTarget().getFiducialId() == 4 
            || getResult(position).getBestTarget().getFiducialId() == 5
            || getResult(position).getBestTarget().getFiducialId() == 9
            || getResult(position).getBestTarget().getFiducialId() == 10))
            {
                Pose3d temp = calculatePoseFromCameraResult(getResult(position), offsets.get(position));
                if(temp != null){
                    double yaw = temp.getRotation().getZ();
                    return yaw;
                }
                
            }
            return null;
    }
    
    // Field coordinates for the april tags (converting inches to meters)
    private void initializeFiducialMap(double inchesToMeters) {
        fiducialMap.put(1, new Pose3d(593.68 * inchesToMeters, 9.68 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(2, new Pose3d(637.21 * inchesToMeters, 34.79 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(3, new Pose3d(652.73 * inchesToMeters, 196.17 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(4, new Pose3d(652.73 * inchesToMeters, 218.42 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(5, new Pose3d(578.77 * inchesToMeters, 323.00 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(270))));
        fiducialMap.put(6, new Pose3d(72.50 * inchesToMeters, 323.00 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(270))));
        fiducialMap.put(7, new Pose3d(-1.50 * inchesToMeters, 218.42 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(8, new Pose3d(-1.50 * inchesToMeters, 196.17 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(9, new Pose3d(14.02 * inchesToMeters, 34.79 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(10, new Pose3d(57.54 * inchesToMeters, 9.68 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(11, new Pose3d(468.69 * inchesToMeters, 146.19 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(300))));
        fiducialMap.put(12, new Pose3d(468.69 * inchesToMeters, 177.10 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(13, new Pose3d(441.74 * inchesToMeters, 161.62 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(14, new Pose3d(209.48 * inchesToMeters, 161.62 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(15, new Pose3d(182.73 * inchesToMeters, 177.10 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(16, new Pose3d(182.73 * inchesToMeters, 146.19 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(240))));
    }
}
