package frc.robot.subsystems.vision;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drivebase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
public class CameraSystem{

    private final Map<Integer, Pose3d> fiducialMap = new HashMap<>();
    private ArrayList<PhotonCamera> cameras;
    private ArrayList<Transform3d> offsets;
    private ArrayList<PhotonPoseEstimator> estimators;
    private ArrayList<Boolean> hasAprilTagDetection;
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public Drivebase drivebase;
    

    private static CameraSystem instance;

    private CameraSystem() {

        double inchesToMeters = 0.0254;
        cameras = new ArrayList<PhotonCamera>();
        offsets  = new ArrayList<Transform3d>();
        estimators = new ArrayList<PhotonPoseEstimator>();
        hasAprilTagDetection = new ArrayList<Boolean>();


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
    public boolean CameraHasAprilTagDetection(int position){
        return hasAprilTagDetection.get(position);
    }
    // adds the camera, offset, and estimator to their arraylists; each camera, offset, and estimator have the same position in each arraylist
    public void AddCamera(PhotonCamera camera, Transform3d offset, boolean hasAprilTagDetection){
        cameras.add(camera);
        offsets.add(offset);
        
        PhotonPoseEstimator estimator = new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, offset);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        estimators.add(estimator);
        this.hasAprilTagDetection.add(hasAprilTagDetection);
    }
    public void AddVisionMeasurements(SwerveDrivePoseEstimator swerveEstimator)
    {
        int cameraCount = 0;
        for (var estimator : estimators) 
        {
            if(CameraHasAprilTagDetection(cameraCount))
            {
                Optional<EstimatedRobotPose> estimatedPose = estimator.update(getResult(cameraCount));
                Matrix<N3,N1> camStdDev = confidenceCalculator(cameraCount);
                if(!estimatedPose.isEmpty() && camStdDev != null)
                {
                    Pose2d camRobotPose = estimatedPose.get().estimatedPose.toPose2d();
                    double timestamp = estimatedPose.get().timestampSeconds;
                    swerveEstimator.addVisionMeasurement(camRobotPose, timestamp, camStdDev);
                }
            }
            cameraCount++;
        }
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
            if(cam.getLatestResult().hasTargets() && CameraHasAprilTagDetection(cameraCount))
            {
                // if the camera picks up a tag, it calculates the position from the tag and runs it through a pose estimator
                Pose3d orig = calculatePoseFromCameraResult(cam.getLatestResult(), offsets.get(cameraCount));
                if(orig != null){
                    // Optional<EstimatedRobotPose> estimatedPose = usePoseEstimator(cameraCount, orig.toPose2d());
                    // if(!estimatedPose.isEmpty()){
                    //     Pose3d temp = estimatedPose.get().estimatedPose;
                    //     // add the components of the pose 3d to the sums
                    //     sumX += temp.getX();
                    //     sumY += temp.getY();
                    //     rotationSumx += temp.getRotation().getX();
                    //     rotationSumY += temp.getRotation().getY();
                    //     rotationSumZ += temp.getRotation().getZ();              
                    //     cameraTagCount++;     
                    // }
                    sumX += orig.getX();
                    sumY += orig.getY();
                    rotationSumx += orig.getRotation().getX();
                    rotationSumY += orig.getRotation().getY();
                    rotationSumZ += orig.getRotation().getZ();              
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
        // estimators.get(position).setReferencePose(prevPose);
        return estimators.get(position).update(getResult(position));
    }
    // calculates the postition of tag to robot from one camera's results 
    private Pose3d calculatePoseFromCameraResult(PhotonPipelineResult result, Transform3d cameraOffset) {
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            MultiTargetPNPResult pnpResult = result.getMultiTagResult();
            
                
            // gets the position of the april tag scanned
            //Pose3d fiducialPose = fiducialMap.get(target.getFiducialId());

            if(pnpResult.estimatedPose.isPresent)
            {
                Transform3d fieldToCamera = pnpResult.estimatedPose.best;
                Pose3d cameraToTargetPose = new Pose3d().transformBy(fieldToCamera);
                Pose3d robotPose3d = cameraToTargetPose.transformBy(cameraOffset);
                return new Pose3d(
                    robotPose3d.getX(),
                    robotPose3d.getY(),
                    robotPose3d.getZ(),
                    robotPose3d.getRotation()
                );

            }

            // if (fiducialPose != null) {
            //     // calcuates the april tag position to the camera
            //     Transform3d transform = target.getBestCameraToTarget().inverse();
            //     Pose3d cameraToTargetPose = fiducialPose.transformBy(transform);
            //     // finds the pose of the robot from the camera position
            //     Pose3d robotPose3d = cameraToTargetPose.transformBy(cameraOffset);
            //     return new Pose3d(
            //         robotPose3d.getX(),
            //         robotPose3d.getY(),
            //         robotPose3d.getZ(),
            //         robotPose3d.getRotation()
            //     );
            // }
        }
        return null;
    }
    private Matrix<N3, N1> confidenceCalculator(int position) 
    {
        Optional<EstimatedRobotPose> estimationPose = usePoseEstimator(position, null);
        if(!estimationPose.isEmpty())
        {
            EstimatedRobotPose estimation = estimationPose.get();
            double smallestDistance = Double.POSITIVE_INFINITY;
            for (var target : estimation.targetsUsed)  
            {
                var t3d = target.getBestCameraToTarget();
                double distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
                if (distance < smallestDistance)
                    smallestDistance = distance;
            }
            double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
            ? 1
            : Math.max(
                1,
                (estimation.targetsUsed.get(0).getPoseAmbiguity()
                    + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
                    * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);
            double confidenceMultiplier = Math.max(
            1,
            (Math.max(
                1,
                Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
                * Constants.VisionConstants.DISTANCE_WEIGHT) * poseAmbiguityFactor)
                / (1 + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

            return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
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
            if(cam.getLatestResult().hasTargets() && CameraHasAprilTagDetection(cameraCount)){
                Pose3d orig = calculatePoseFromCameraResult(cam.getLatestResult(), offsets.get(cameraCount));
                if(orig != null){
                    Optional<EstimatedRobotPose> estimatedPose = usePoseEstimator(cameraCount, orig.toPose2d());
                    if(estimatedPose != null && !estimatedPose.isEmpty())
                        return estimatedPose.get().timestampSeconds;
                }
                
            }
            cameraCount++;
        }
        return -1;
    }
    // returns a Double Object, so need check if it is null
    public Double getYawForTag(int position, int ID){
            if(getResult(position).hasTargets() && CameraHasAprilTagDetection(position))
            {
                List<PhotonTrackedTarget> targets = getResult(position).getTargets();
                for(var target : targets)
                {
                    if(target != null && target.getFiducialId() == ID)
                    {
                        return target.getYaw();
                    }
                }
            } 
            // else if(getResult(position).hasTargets() && getResult(position).getBestTarget().getFiducialId() == 3){
            //     List<PhotonTrackedTarget> targets = getResult(position).getTargets();
            //     for(PhotonTrackedTarget target : targets){
            //         if(target.getFiducialId() == 4){
            //             return target.getYaw();
            //         }
            //     }
            // }
            return null;
    } 
    public Double getTargetRange(int position, int ID){
        Double targetRange = null;
        // if(getResult(position).hasTargets() && getResult(position).getBestTarget().getFiducialId() == 4){
        //     targetRange = PhotonUtils.calculateDistanceToTargetMeters(-offsets.get(position).getZ(), 57.13 * 0.0254, -offsets.get(position).getRotation().getY(), Units.degreesToRadians(getResult(position).getBestTarget().getPitch()));
        // }
        // else if(getResult(position).hasTargets() && getResult(position).getBestTarget().getFiducialId() == 3){
        //     List<PhotonTrackedTarget> targets = getResult(position).getTargets();
        //         for(PhotonTrackedTarget target : targets){
        //             if(target.getFiducialId() == 4){
        //                 targetRange = PhotonUtils.calculateDistanceToTargetMeters(-offsets.get(position).getZ(), 57.13 * 0.0254, -offsets.get(position).getRotation().getY(), Units.degreesToRadians(getResult(position).getBestTarget().getPitch()));
        //             }
        //         }
        // }
        
            List<PhotonTrackedTarget> targets = getResult(position).getTargets();
            for(PhotonTrackedTarget target : targets)
            {
               if(target.getFiducialId() == ID)
               {
                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(-offsets.get(position).getZ(), 
                    aprilTagFieldLayout.getTagPose(ID).get().getZ(), 
                    offsets.get(position).getRotation().getY(), 
                    Units.degreesToRadians(target.getPitch()));
                } 
            }

        return targetRange;
    }
    /** calcuates the angle the launcher should be at to make it in the speaker; alhtough this mehtod does take in an ID to make it more
     generic, it generally should only run on april tag #4, which is the one for the speaker
    */ 
    public Double getTargetAngle(int position, int ID)
    {
        Double targetAngle = null;
        List<PhotonTrackedTarget> targets = getResult(position).getTargets();
        for(PhotonTrackedTarget target : targets)
        {
            if(target.getFiducialId() == ID)
            {
                Double base = getTargetRange(position, ID);
                double height = aprilTagFieldLayout.getTagPose(ID).get().getZ() + .44 + Constants.LauncherConstants.launcherPivotHeight;
                Double angleInRadians = null;
                if(base != null)
                {
                    angleInRadians = Math.atan(height/base);
                }
                if(angleInRadians != null)
                {
                    // This equation was found by gathering many angles and looking at their PID value and running a quadratic regression
                    targetAngle = (-2.441919181 * Math.pow(angleInRadians, 2)) + (-26.50060644 * angleInRadians) + 5.23837215;
                }
            }
        }
        return targetAngle;
    }
    // Gets the Yaw for the scanned object; should turn towards the object
    public Double getYawforObject(int position)
    {
        if(CameraHasAprilTagDetection(position) && getResult(position).hasTargets())
        {
            var target = getResult(position).getBestTarget();
            return target.getYaw();
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
