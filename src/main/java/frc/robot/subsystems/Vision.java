package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.visionData;

//https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html
//READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


public class Vision extends SubsystemBase {

    //make sure the name in quotes is EXACTLY the same as it is in PV
    PhotonCamera LeftCamera = new PhotonCamera("USB CAM 1 (High)");
    PhotonCamera RightCamera = new PhotonCamera("USB CAM 2 (High)");
    public Pose2d RobotPose;
    public Pose2d targetPose;
    public double yaw;
    public double pitch;
    public double area;
    public double skew;
    public double distance;

    public Optional<Pose3d> targetpose;
    public double targetheight;
    public int targetID;
    public double poseAmbiguity;
    public Transform3d bestCameraToTarget;
    public Transform3d alternateCameraToTarget;
    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, visionData.robotToCamLeft);    

    public Vision(){
        //Getting target data

        //"A PhotonPipelineResult is a container that contains all information about currently detected targets from a PhotonCamera. 
        

        /*
         * you must always check if the result has a target via hasTargets()/HasTargets() before getting targets or else you may get a null pointer exception.
         * What is a Photon Tracked Target?
         * A tracked target contains information about each target from a pipeline result. This information includes yaw, pitch, area, and robot relative pose.
         */

        

        /*
         * double getYaw() = yaw of target in degrees (positive right)
         * double getPitch() = pitch of target in degress (positive up)
         * double getArea() = the area( how much of the camera feed the bounding box takes up) as a percent
         * double[] getCorners() = the 4 corners of the miniumum bounding box rectangle
         * Transform2d getCameraToTarget() = the camera to target transform, see 2d transform documentation
         */

         // Get information from target.
        
        

        /*
         * note:
         * you can also get fiducial id, pose ambiguity, best camear to target, and alternate camera to target
         * 
         */
        

        //------------------------------------------------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------
        //using target data

        // Calculate robot's field relative pose (for use in updating robot pose)


        /*
        //traditional way of caluclating pose:
        // Calculate robot's field relative pose
        // might need to be calculated in drive subsystem, because of the gyro and
        // setting pose update
        Pose2D robotPose = PhotonUtils.estimateFieldToRobot(
        kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-besttarget.getYaw()), gyro.getRotation2d(), targetPose, visionData.robotToCamLeft);
        */

        

        
        

        //get yaw towards target, i.e. hub from 2022 rapid react
        //Rotation2d targetYaw = PhotonUtils.getYawToPose(RobotPose, targetPose);

        // Construct PhotonPoseEstimator
        //photon vision docs says this needs the camera as an argument, but this doesn't want it, take note of this
        //addvisionmeasurement...?

        



        


    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches, etc
        //You can retrieve the latest pipeline result using the PhotonCamera instance."
        //TODO: make stereo vision work?!?!
        //maybe just use one camera or what idk :sob:
    
        if(LeftCamera.isConnected()){
            var result = LeftCamera.getLatestResult();

        // Check if the latest result has any targets.
        //boolean hasTargets = result.hasTargets();

        // Get a list of currently tracked targets.
        //List<PhotonTrackedTarget> targets = result.getTargets();

        // Get the current best target.
            if(result.hasTargets()){
                PhotonTrackedTarget besttarget = result.getBestTarget();
                yaw = besttarget.getYaw();
                pitch = besttarget.getPitch();
                area = besttarget.getArea();
                skew = besttarget.getSkew();

                targetID = besttarget.getFiducialId();
                targetpose = aprilTagFieldLayout.getTagPose(targetID);
                targetheight = targetpose.get().getMeasureZ().abs(Meter);
                poseAmbiguity = besttarget.getPoseAmbiguity();
                bestCameraToTarget = besttarget.getBestCameraToTarget(); //lowest error transform
                alternateCameraToTarget = besttarget.getAlternateCameraToTarget(); //highest error transform
                //robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                //besttarget.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(besttarget.getFiducialId()).get(), visionData.robotToCamLeft);
                //distance equation:
                //distance = (targetheight - cameraheight) / tan(cameraangle + Ty)
                //THIS IS DISTANCE IN METERS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                distance = (targetheight - visionData.leftCamHeight) / Math.tan(visionData.leftCamAngle.getAngle() + yaw);


                var value = LeftCamera.getLatestResult();
                double value2 = value.getBestTarget().yaw;
                SmartDashboard.putNumber("SMARTDASHBOARDYAW", value2); //set zero if value2 does not exist //TODO:
            }

        }
        

        //calculate distance given pose
        //double distanceToTarget = PhotonUtils.getDistanceToPose(RobotPose, targetPose);

        // Calculate a translation from the camera to the target.
        //needs distance
        //Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
        //distanceToTarget, Rotation2d.fromDegrees(-besttarget.getYaw()));


        //-------------------------------------------------
        //smartdashboardstuff

        SmartDashboard.putNumber("TargetYaw", yaw);
        SmartDashboard.putNumber("TargetPitch", pitch);
        SmartDashboard.putNumber("TargetArea", area);
        SmartDashboard.putNumber("TargetSkew", skew);
        SmartDashboard.putNumber("TargetID", targetID);
        SmartDashboard.putNumber("TargetPoseAmbiguity", poseAmbiguity);

        
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

    public Double getYaw(){
        if(LeftCamera.isConnected() || RightCamera.isConnected()){
            var value = LeftCamera.getLatestResult();
            double yaw = value.getBestTarget().yaw;
            return yaw;
        }
        return null;
    }


    public Double getPitch(String camera){
        if(LeftCamera.isConnected() || RightCamera.isConnected()){
            var value = LeftCamera.getLatestResult();
            double yaw = value.getBestTarget().pitch;
            return yaw;
        }
        return null;
    }


    public Double getArea(String camera){
        if(LeftCamera.isConnected() || RightCamera.isConnected()){
            var value = LeftCamera.getLatestResult();
            double yaw = value.getBestTarget().area;
            return yaw;
        }
        return null;
    }


    public Double getSkew(String camera){
        if(LeftCamera.isConnected() || RightCamera.isConnected()){
            var value = LeftCamera.getLatestResult();
            double yaw = value.getBestTarget().skew;
            return yaw;
        }
        return null;
    }


    public int getID(){
        return targetID;
    }

    //public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        
       // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
       // return photonPoseEstimator.update(result);
   // }

}
