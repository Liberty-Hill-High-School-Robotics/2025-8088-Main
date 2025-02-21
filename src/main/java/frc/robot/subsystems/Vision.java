package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.visionData;

//https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html
//READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


public class Vision extends SubsystemBase {

    //make sure the name in quotes is EXACTLY the same as it is in PV
    PhotonCamera LeftCamera = new PhotonCamera("USB CAM 1 (High)");
    PhotonCamera RightCamera = new PhotonCamera("USB CAM 2 (High)");
    public final Pigeon2 m_gyro = new Pigeon2(CanIDs.GyroID);

    /*
    public Optional<EstimatedRobotPose> robotPose;
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
    */


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
        /*
         * note:
         * you can also get fiducial id, pose ambiguity, best camear to target, and alternate camera to target
         * 
         */
        

        //------------------------------------------------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------
        //------------------------------------------------------------------------------------------------------
        //using target data



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


        // Check if the latest result has any targets.
        //boolean hasTargets = result.hasTargets();
        // Get a list of currently tracked targets.
        //List<PhotonTrackedTarget> targets = result.getTargets();
        //distance equation:
        //distance = (targetheight - cameraheight) / tan(cameraangle + Ty)
    

        //LEFTCAMERA VALUES
        if(LeftCamera.isConnected()){
            var Lresult = LeftCamera.getLatestResult();
            // Get the current best target.
            if(Lresult.hasTargets()){
                PhotonTrackedTarget Lbesttarget = Lresult.getBestTarget();
                double Lyaw = Lbesttarget.getYaw();
                double Lpitch = Lbesttarget.getPitch();
                double Larea = Lbesttarget.getArea();
                double Lskew = Lbesttarget.getSkew();

                int LtargetID = Lbesttarget.getFiducialId();
                Optional<Pose3d> Ltargetpose = aprilTagFieldLayout.getTagPose(LtargetID);
                double Ltargetheight = Ltargetpose.get().getMeasureZ().abs(Meter);
                double LposeAmbiguity = Lbesttarget.getPoseAmbiguity();
                Transform3d LbestCameraToTarget = Lbesttarget.getBestCameraToTarget(); //lowest error transform
                Transform3d LalternateCameraToTarget = Lbesttarget.getAlternateCameraToTarget(); //highest error transform

                Pose2d Ltargetpose2D = Ltargetpose.get().toPose2d();
                //Pose Estimate
                //target pitch == 0
                //camera pitch == check constants
                Pose2d robotPoseL = PhotonUtils.estimateFieldToRobot(        
                visionData.leftCamHeight, Ltargetheight, visionData.leftCamPitch, (Lbesttarget.pitch * (Math.PI/180)),
                Rotation2d.fromDegrees(-Lbesttarget.getYaw()), m_gyro.getRotation2d(), Ltargetpose2D, visionData.robotToCamLeft2D);
                SmartDashboard.putNumber("poseLX", robotPoseL.getX());
                SmartDashboard.putNumber("poseLY", robotPoseL.getY());
                SmartDashboard.putNumber("poseLR", robotPoseL.getRotation().getDegrees());


                SmartDashboard.putNumber("poseLXT", Ltargetpose.get().getX());
                SmartDashboard.putNumber("poseLYT", Ltargetpose.get().getY());
                SmartDashboard.putNumber("poseLRT", Ltargetpose.get().getRotation().getAngle());

                if (Lresult.getMultiTagResult().isPresent()){

                }


            }
        }

        //RIGHTCAMERA VALUES
        if(RightCamera.isConnected()){
            var Rresult = LeftCamera.getLatestResult();
            // Get the current best target.
            if(Rresult.hasTargets()){
                PhotonTrackedTarget Rbesttarget = Rresult.getBestTarget();
                double Ryaw = Rbesttarget.getYaw();
                double Rpitch = Rbesttarget.getPitch();
                double Rarea = Rbesttarget.getArea();
                double Rskew = Rbesttarget.getSkew();

                int RtargetID = Rbesttarget.getFiducialId();
                Optional<Pose3d> Rtargetpose = aprilTagFieldLayout.getTagPose(RtargetID);
                double Rtargetheight = Rtargetpose.get().getMeasureZ().abs(Meter);
                double RposeAmbiguity = Rbesttarget.getPoseAmbiguity();
                Transform3d RbestCameraToTarget = Rbesttarget.getBestCameraToTarget(); //lowest error transform
                Transform3d RalternateCameraToTarget = Rbesttarget.getAlternateCameraToTarget(); //highest error transform
                Pose2d Rtargetpose2D = Rtargetpose.get().toPose2d();


                double valeue = PhotonUtils.calculateDistanceToTargetMeters(visionData.rightCamHeight, Rtargetheight, visionData.rightCamPitch, (Rbesttarget.pitch * (Math.PI/180)));
                valeue = valeue * 1000;
                SmartDashboard.putNumber("distanceValue", valeue);
                var relative = PhotonUtils.estimateCameraToTargetTranslation(valeue, Rbesttarget.bestCameraToTarget.getRotation().toRotation2d());
                SmartDashboard.putNumber("cameratotargetX", relative.getX());
                SmartDashboard.putNumber("cameratotargetY", relative.getY());
                SmartDashboard.putNumber("cameratotargetA", relative.getAngle().getDegrees());




                Pose2d robotPoseR = PhotonUtils.estimateFieldToRobot(   
                     
                visionData.rightCamHeight, Rtargetheight, visionData.rightCamPitch, (Rbesttarget.pitch * (Math.PI/180)),
                Rotation2d.fromDegrees(-Rbesttarget.getYaw()), m_gyro.getRotation2d(), Rtargetpose2D, visionData.robotToCamLeft2D);

                SmartDashboard.putNumber("poseRX", robotPoseR.getX());
                SmartDashboard.putNumber("poseRY", robotPoseR.getY());
                SmartDashboard.putNumber("poseRR", robotPoseR.getRotation().getDegrees());
                SmartDashboard.putNumber("poseRXT", Rtargetpose.get().getX());
                SmartDashboard.putNumber("poseRYT", Rtargetpose.get().getY());
                SmartDashboard.putNumber("poseRRT", Rtargetpose.get().getRotation().getAngle());
                SmartDashboard.putNumber("angledegrese", (Rtargetpose.get().getRotation().getAngle()) * (180/Math.PI));


            }
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


    public Optional<EstimatedRobotPose> getPoseVision(){
        return photonPoseEstimator.update(LeftCamera.getLatestResult());
    }


    //public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        
       // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
       // return photonPoseEstimator.update(result);
   // }

}
