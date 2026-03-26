package frc.robot;


import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.SimCameraProperties;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;


public class Vision extends SubsystemBase {
    
    private static final PhotonCamera camera1 = new PhotonCamera(VisionConstants.kCameraName1);
    private static final PhotonCamera camera2 = new PhotonCamera(VisionConstants.kCameraName2);
    // private static final PhotonCamera camera3 = new PhotonCamera(VisionConstants.kCameraName3);


    private static VisionSystemSim visionSim;
    private static PhotonCameraSim cameraSim1;
    private static PhotonCameraSim cameraSim2;
    private Supplier<Pose2d> drivePoseSupplier;


    private static PhotonPipelineResult result1 = null;
    private static PhotonPipelineResult result2 = null;
    private static PhotonPipelineResult result3 = null;

    private static final PhotonPoseEstimator poseEstimator1 = new PhotonPoseEstimator(
        VisionConstants.kTagLayout, VisionConstants.kRobotToCam1);
    private static final PhotonPoseEstimator poseEstimator2 = new PhotonPoseEstimator(
        VisionConstants.kTagLayout, VisionConstants.kRobotToCam2);
    private static final PhotonPoseEstimator poseEstimator3 = new PhotonPoseEstimator(
        VisionConstants.kTagLayout, VisionConstants.kRobotToCam3);

    public Vision(Supplier<Pose2d> drivePoseSupplier){
        this.drivePoseSupplier = drivePoseSupplier;
        if (!RobotBase.isReal()) {
            visionSim = new VisionSystemSim("main");

            // Camera properties (tune if needed)
            SimCameraProperties props = new SimCameraProperties();
            props.setCalibration(320, 240, Rotation2d.fromDegrees(91)); // resolution + FOV
            props.setFPS(30);
            props.setAvgLatencyMs(0);
            props.setLatencyStdDevMs(5);

            cameraSim1 = new PhotonCameraSim(camera1, props);

            cameraSim1.enableDrawWireframe(true);

            // Add camera to sim with robot-to-camera transform
            visionSim.addCamera(cameraSim1, VisionConstants.kRobotToCam1);

            cameraSim2 = new PhotonCameraSim(camera2, props);

            visionSim.addCamera(cameraSim2, VisionConstants.kRobotToCam2);

            // Add AprilTags
            visionSim.addAprilTags(VisionConstants.kTagLayout);
        }
    }

    @Override
    public void periodic() {

        if (!Robot.isReal()) {
            Pose2d robotPose = drivePoseSupplier.get();

            visionSim.update(robotPose);
        }
        
        var results1 = camera1.getAllUnreadResults();
        if (!results1.isEmpty()){
            result1 = results1.get(results1.size() - 1);
        }
        var results2 = camera2.getAllUnreadResults();
        if (!results2.isEmpty()){
            result2 = results2.get(results2.size() - 1);
        }
        // var results3 = camera3.getAllUnreadResults();
        // if (!results3.isEmpty()){
        //     result3 = results3.get(results3.size() - 1);
        // }
    }

    public static PhotonPipelineResult getResult1() {
        return result1;
    }

    public static PhotonPipelineResult getResult2() {
        return result2;
    }

    public static PhotonPipelineResult getResult3() {
        return result3;
    }

    public static PhotonCamera getCamera1() {
        return camera1;
    }

    public static PhotonCamera getCamera2() {
        return camera2;
    }

    // public static PhotonCamera getCamera3() {
    //     return camera3;
    // }

    public static boolean resultHasTargets() {
        return (result1 != null && result1.hasTargets()) || (result2 != null && result2.hasTargets()) || (result3 != null && result3.hasTargets());
    }

    public static int[] tagsInFrame() {
        List<PhotonTrackedTarget> targets = getAllTargets();
        int[] tags = new int[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            tags[i] = targets.get(i).getFiducialId();
        }
        return tags;
    }

    public static List<PhotonTrackedTarget> getAllTargets() {
        List<PhotonTrackedTarget> allTargets = new ArrayList<>();
        if (result1 != null && result1.hasTargets()) {
            allTargets.addAll(result1.getTargets());
        }
        // if (result2 != null && result2.hasTargets()) {
        //     allTargets.addAll(result2.getTargets());
        // }
        // if (result3 != null && result3.hasTargets()) {
        //     allTargets.addAll(result3.getTargets());
        // }
        return allTargets;
    }

    public static int getBestTag() {
        if (result1 != null && result1.hasTargets()) {
            return result1.getBestTarget().getFiducialId();
        }
        if (result2 != null && result2.hasTargets()) {
            return result2.getBestTarget().getFiducialId();
        }
        return 0;
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam1(PhotonPipelineResult result, Pose2d referencePose) {
        if (result == null || !result.hasTargets()){
            return Optional.empty();
        }

        Pose3d[] usedTags = new Pose3d[result.targets.size()];
        for (int i = 0; i < result.targets.size(); i++){
          usedTags[i] = (VisionConstants.kTagLayout.getTagPose(result.targets.get(i).fiducialId).get());
        }

        Logger.recordOutput("PoseEst/Camera 1 Tags Used", usedTags);

        Optional<EstimatedRobotPose> update = poseEstimator1.estimateClosestToReferencePose(result, new Pose3d(referencePose));
            
        
        return update;
    }

    public static Matrix<N3, N1> updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        Matrix<N3, N1> curStdDevs = VisionConstants.kSingleTagStdDevs;
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;
        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = VisionConstants.kTagLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible.
                curStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 2)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                // TODO Tweak the constant here PLEASE to change how much we trust multi tag as distances increase
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
        return curStdDevs;
    }



    public static Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam2(Pose2d prevEstimatedRobotPose, PhotonPipelineResult result) {
        if (result == null || !result.hasTargets()){
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> update = Optional.empty();

        Pose3d[] usedTags = new Pose3d[result.targets.size()];
        for (int i = 0; i < result.targets.size(); i++){
          usedTags[i] = (VisionConstants.kTagLayout.getTagPose(result.targets.get(i).fiducialId).get());
        }

        Logger.recordOutput("PoseEst/Camera 2 Tags Used", usedTags);

        
        update = poseEstimator2.estimateClosestToCameraHeightPose(result);
        
        return update;
    }

    public static Optional<EstimatedRobotPose> getEstimatedGlobalPoseCam3(Pose2d prevEstimatedRobotPose, PhotonPipelineResult result) {
        if (result == null || !result.hasTargets()){
            return Optional.empty();
        }

        Optional<EstimatedRobotPose> update = Optional.empty();
        if (result.getTargets().size() > 1){
            update = poseEstimator3.estimateCoprocMultiTagPose(result);
        }
        else {
            update = poseEstimator3.estimateLowestAmbiguityPose(result);
        }
        
        
        return update;
    }

    public static double targetYaw(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target.getYaw();
            }
        }
        return 0;
    }

    public static Transform3d targetTransform(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target.getBestCameraToTarget();
            }
        }
        return new Transform3d();
    }

    public static PhotonTrackedTarget returnTag(int targetNumber) {
        for (PhotonTrackedTarget target : getAllTargets()) {
            if (target.getFiducialId() == targetNumber) {
                return target;
            }
        }
        return new PhotonTrackedTarget(); // Empty target
    }
}