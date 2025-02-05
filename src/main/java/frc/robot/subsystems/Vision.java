package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.visionData;

//https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html
//READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


public class Vision extends SubsystemBase {

    //make sure the name in quotes is EXACTLY the same as it is in PV
    PhotonCamera LeftCamera = new PhotonCamera("LeftCamera");
    PhotonCamera RightCamera = new PhotonCamera("RightCamera");
    Pose2d RobotPose;
    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    

    public Vision(){
        //Getting target data

        //"A PhotonPipelineResult is a container that contains all information about currently detected targets from a PhotonCamera. 
        //You can retrieve the latest pipeline result using the PhotonCamera instance."
        var result = LeftCamera.getLatestResult();

        // Check if the latest result has any targets.
        boolean hasTargets = result.hasTargets();

        /*
         * you must always check if the result has a target via hasTargets()/HasTargets() before getting targets or else you may get a null pointer exception.
         * What is a Photon Tracked Target?
         * A tracked target contains information about each target from a pipeline result. This information includes yaw, pitch, area, and robot relative pose.
         */

        // Get a list of currently tracked targets.
        List<PhotonTrackedTarget> targets = result.getTargets();

        // Get the current best target.
        PhotonTrackedTarget besttarget = result.getBestTarget();

        /*
         * double getYaw() = yaw of target in degrees (positive right)
         * double getPitch() = pitch of target in degress (positive up)
         * double getArea() = the area( how much of the camera feed the bounding box takes up) as a percent
         * double[] getCorners() = the 4 corners of the miniumum bounding box rectangle
         * Transform2d getCameraToTarget() = the camera to target transform, see 2d transform documentation
         */

         // Get information from target.
        double yaw = besttarget.getYaw();
        double pitch = besttarget.getPitch();
        double area = besttarget.getArea();
        double skew = besttarget.getSkew();

        /*
         * note:
         * you can also get fiducial id, pose ambiguity, best camear to target, and alternate camera to target
         * 
         */
        int targetID = besttarget.getFiducialId();
        double poseAmbiguity = besttarget.getPoseAmbiguity();
        Transform3d bestCameraToTarget = besttarget.getBestCameraToTarget(); //lowest error transform
        Transform3d alternateCameraToTarget = besttarget.getAlternateCameraToTarget(); //highest error transform

        //------------------------------------------------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------
        //using target data

        // Calculate robot's field relative pose (for use in updating robot pose)
        if (aprilTagFieldLayout.getTagPose(besttarget.getFiducialId()).isPresent()) {
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
            besttarget.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(besttarget.getFiducialId()).get(), visionData.robotToCamLeft);
        }

        /*
        //traditional way of caluclating pose:
        // Calculate robot's field relative pose
        // might need to be calculated in drive subsystem, because of the gyro and
        // setting pose update
        Pose2D robotPose = PhotonUtils.estimateFieldToRobot(
        kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-besttarget.getYaw()), gyro.getRotation2d(), targetPose, visionData.robotToCamLeft);
        */

        //calculate distance given pose
        double distanceToTarget = PhotonUtils.getDistanceToPose(RobotPose, targetPose);

        // Calculate a translation from the camera to the target.
        //needs distance
        Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
        distanceToTarget, Rotation2d.fromDegrees(-besttarget.getYaw()));

        //get yaw towards target, i.e. hub from 2022 rapid react
        Rotation2d targetYaw = PhotonUtils.getYawToPose(RobotPose, targetPose);

        // Construct PhotonPoseEstimator
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, visionData.robotToCamLeft);
        //photon vision docs says this needs the camera as an argument, but this doesn't want it, take note of this
        //addvisionmeasurement...?

        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimator.update(result);
    }

        


    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        //Mostly used for debug and such
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
    // Should include run/stop/run back, etc.

    //as well as check for limits and reset encoders,
    //return true/false if limit is true, or encoder >= x value

    public void ___(){
        //motor.set(PID.calculate(position, setpoint));
        //motor.set(number);
        
    }

}
