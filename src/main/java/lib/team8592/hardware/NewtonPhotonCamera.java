package lib.team8592.hardware;

import java.util.List;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public class NewtonPhotonCamera {
    private PhotonCamera camera;

    public NewtonPhotonCamera(String name) {
        this.camera = new PhotonCamera(name);
    }

    /**
     * Grabs all visible targets
     */
    public List<PhotonTrackedTarget> getTargets() {
        return this.camera.getAllUnreadResults().get(0).targets;
    }

    /**
     * Grabs the target most matching the current pipeline
     */
    public PhotonTrackedTarget getBestTarget() {
        return getTargets().get(0);
    }

    /**
     * Sets the current pipeline index
     */
    public void setPipeline(int pipeline) {
        this.camera.setPipelineIndex(pipeline);
    }

    /**
     * Whether there is any target visible in the camera
     */
    public boolean isAnyTargetVisible() {
        return this.getBestTarget() != null;
    }

    /**
     * Whether a particular AprilTag is visible in the camera
     */
    public boolean isAprilTagVisible(int id) {
        for (PhotonTrackedTarget target : getTargets()) {
            if (target.getFiducialId() == id) return true;
        }
        return false;
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToBestTarget() {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(this.getBestTarget().getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToTarget(PhotonTrackedTarget target) {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(target.getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToAprilTag(int id) {
        for (PhotonTrackedTarget target : getTargets()) {
            if (target.getFiducialId() == id) return Rotation2d.fromDegrees(target.getYaw());
        }
        return null; // Return null if the particular april tag is not seen by the camera
    }

    /**
     * The calculated 2d field position of the robot based on april tag vision data
     */
    public Pose2d getEstimatedFieldPose() {
        return PhotonUtils.estimateFieldToRobotAprilTag(
            getBestTarget().getBestCameraToTarget(), 
            Constants.VISION.APRIL_TAG_LAYOUT.getTagPose(
                getBestTarget().getFiducialId()
            ).get(), 
            Constants.VISION.CAMERA_POSE_TO_ROBOT_POSE
        ).toPose2d();
    }
}