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
import java.util.logging.Logger;
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
    public PhotonPipelineResult getResult(int position){
        return cameras.get(position).getLatestResult();
    }
    public void AddCamera(PhotonCamera camera, Transform3d offset){
        cameras.add(camera);
        offsets.add(offset);
        estimators.add(new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, offset));
    }
     public Pose2d calculateRobotPosition() {
        int count = 0;
        int cameraTagCount = 0;
        double sumX = 0;
        double sumY = 0;
        double sumZ = 0;
        
        double rotationSumx = 0;
        double rotationSumY = 0;
        double rotationSumZ = 0;
        
        while(count<cameras.size())
        {
            if(getResult(count).hasTargets())
            {
                Pose3d temp = calculatePoseFromCameraResult(getResult(count), offsets.get(count))
                sumX = temp.getX();
                sumY = temp.getY();
                sumZ = temp.getZ();
                rotationSumx = temp.getRotation().getX();
                rotationSumY = temp.getRotation().getY();
                rotationSumZ = temp.getRotation().getZ();              
                cameraTagCount++;     
                
            }
            count++;
        }
        
        
        // Return the Pose2d with available data, or a default Pose2d if none is available
        return frontRobotPose != null ? new Pose2d(frontRobotPose.getX(), frontRobotPose.getY(), new Rotation2d(frontRobotPose.getRotation().getZ())) :
               backRobotPose != null ? new Pose2d(backRobotPose.getX(), backRobotPose.getY(), new Rotation2d(backRobotPose.getRotation().getZ())) :
               new Pose2d();
    }
    private Pose3d calculatePoseFromCameraResult(PhotonPipelineResult result, Transform3d cameraOffset) {
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            // Optional<Pose3d> aprilPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            // Transform3d transformPhoton = target.getBestCameraToTarget();
            // Pose3d robotToTag = PhotonUtils.estimateFieldToRobotAprilTag(transformPhoton, aprilPose.get(), cameraOffset);
            // return robotToTag;
                
            
            Pose3d fiducialPose = fiducialMap.get(target.getFiducialId());

            if (fiducialPose != null) {
                Transform3d transform = target.getBestCameraToTarget().inverse();
                Pose3d cameraToTargetPose = fiducialPose.transformBy(transform);

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
    // Field coordinates for the april tags (meters)
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
