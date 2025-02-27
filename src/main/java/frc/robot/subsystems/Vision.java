package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.visionData;

//https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html
//READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


public class Vision extends SubsystemBase {

    //make sure the name in quotes is EXACTLY the same as it is in PV
    PhotonCamera LeftCamera = new PhotonCamera("USB CAM L (High)");
    PhotonCamera RightCamera = new PhotonCamera("USB CAM R (High)");
    public final Pigeon2 m_gyro = new Pigeon2(CanIDs.GyroID);

    public final double xLeftOffset = visionData.robotToCamLeft.getY();
    public final double xRightOffset = visionData.robotToCamRight.getY();

    public final double xDistance = 0.356;
    double D = 0;
    boolean same = false;
    double output;
    double outputy;
    double calcYaw = 0;




    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public Vision(){

    }


    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches, etc
        //You can retrieve the latest pipeline result using the PhotonCamera instance."
        // Check if the latest result has any targets.
        //boolean hasTargets = result.hasTargets();
        // Get a list of currently tracked targets.
        //List<PhotonTrackedTarget> targets = result.getTargets();

        /*
         * Get values and data from both cameras, provided they exist (code WILL crash if you dont check)
         * Take values and calculate distance, needed yaw of target and dis. between cameras
         * Tan(YAW) * (Dis/2) = D
         * D = distance of a horizontal line between center of A.T. and the centerpoint between both cameras
         * This math MIGHT work, needs to be tested (ive been thinking about it and it truly is math)
         * HYP should be distance straight line, not aligned to X axis (top down view)
         * 
         */
    

        //LEFTCAMERA VALUES
        if(LeftCamera.isConnected()){
            var Lresult = LeftCamera.getLatestResult();
            // Get the current best target.

            if(Lresult.hasTargets()){

                //get all usable data from target:
                //yaw, pitch, area, ID, pose, height
                PhotonTrackedTarget Lbesttarget = Lresult.getBestTarget();
               // double Lyaw = Lbesttarget.getYaw();
               // double Lpitch = Lbesttarget.getPitch();
                //double Larea = Lbesttarget.getArea();
                int LtargetID = Lbesttarget.getFiducialId();
                Optional<Pose3d> Ltargetpose = aprilTagFieldLayout.getTagPose(LtargetID);
                double Ltargetheight = Ltargetpose.get().getMeasureZ().abs(Meter);
                Pose2d Ltargetpose2D = Ltargetpose.get().toPose2d();
                

                Pose2d robotPoseL = PhotonUtils.estimateFieldToRobot(        
                visionData.leftCamHeight, Ltargetheight, visionData.leftCamPitch, (Lbesttarget.pitch * (Math.PI/180)),
                Rotation2d.fromDegrees(-Lbesttarget.getYaw()), m_gyro.getRotation2d(), Ltargetpose2D, visionData.robotToCamLeft2D);
                SmartDashboard.putNumber("poseLX", robotPoseL.getX());
                SmartDashboard.putNumber("poseLY", robotPoseL.getY());
                SmartDashboard.putNumber("poseLR", robotPoseL.getRotation().getDegrees());
                SmartDashboard.putNumber("poseLXT", Ltargetpose.get().getX());
                SmartDashboard.putNumber("poseLYT", Ltargetpose.get().getY());
                SmartDashboard.putNumber("poseLRT", Ltargetpose.get().getRotation().getAngle());
            }
        }

        //RIGHTCAMERA VALUES
        if(RightCamera.isConnected()){
            var Rresult = LeftCamera.getLatestResult();
            // Get the current best target.

            if(Rresult.hasTargets()){

                //get all usable data from target:
                //yaw, pitch, area, ID, pose, height
                PhotonTrackedTarget Rbesttarget = Rresult.getBestTarget();
                //double Ryaw = Rbesttarget.getYaw();
                //double Rpitch = Rbesttarget.getPitch();
                //double Rarea = Rbesttarget.getArea();
                int RtargetID = Rbesttarget.getFiducialId();
                Optional<Pose3d> Rtargetpose = aprilTagFieldLayout.getTagPose(RtargetID);
                double Rtargetheight = Rtargetpose.get().getMeasureZ().abs(Meter);
                Pose2d Rtargetpose2D = Rtargetpose.get().toPose2d();
                
                Pose2d robotPoseR = PhotonUtils.estimateFieldToRobot(   
                visionData.rightCamHeight, Rtargetheight, visionData.rightCamPitch, (Rbesttarget.pitch * (Math.PI/180)),
                Rotation2d.fromDegrees(-Rbesttarget.getYaw()), m_gyro.getRotation2d(), Rtargetpose2D, visionData.robotToCamLeft2D);
                SmartDashboard.putNumber("poseRX", robotPoseR.getX());
                SmartDashboard.putNumber("poseRY", robotPoseR.getY());
                SmartDashboard.putNumber("poseRR", robotPoseR.getRotation().getDegrees());
                SmartDashboard.putNumber("poseRXT", Rtargetpose.get().getX());
                SmartDashboard.putNumber("poseRYT", Rtargetpose.get().getY());
                SmartDashboard.putNumber("poseRRT", Rtargetpose.get().getRotation().getAngle());
            }
        }


        //distance estimator
        if(RightCamera.isConnected() && LeftCamera.isConnected()){
            var Rresult = RightCamera.getLatestResult();
            var Lresult = LeftCamera.getLatestResult();

            if(Rresult.hasTargets() && Lresult.hasTargets()){

                var LID = Lresult.getBestTarget().fiducialId;
                var RID = Rresult.getBestTarget().fiducialId;
                //check if targetID same
                if(RID != LID){
                    SmartDashboard.putString("targetsame", "NO");
                }
                //if targets are same //TODO
                else{
                SmartDashboard.putString("targetsame", "YES");
                double yawR = Rresult.getBestTarget().yaw;
                double yawL = Lresult.getBestTarget().yaw;
                double yawRB = Rresult.getBestTarget().yaw;
                double yawLB = Lresult.getBestTarget().yaw;
                double c;
                double A;
                double Y;
                double yawC = m_gyro.getYaw().getValueAsDouble();
                yawC = Math.abs(yawC % 360);
                //get yaw on a scale of 0-360
                if(yawC < 90 && yawC > 0){
                    yawC = Math.abs(90 - yawC);
                    //get difference to 90 if yaw between 0 and 90
                }
                else{
                    yawC = yawC % 90;
                    //return otherwise if not between 0 and 90
                }

                /*
                * Math process:
                * First, normalize yaw values
                * calculate (gyro rotation - pose rotation)
                * yaw = yaw - calculated value
                * ^ that will normalize yaw so it works if chassis is not 90­°
                * 
                * take ASA (angle side angle), solve for missing angle
                * plug into law of sines
                * A/sin(a) = B/sin(b) = C/sin(c)
                * 
                * solve for missing side, either one
                * tan(a) * A = D
                * or
                * tan(b) * B = D
                * assuming C is the known side
                //Y = (tan(yaw)) / x
                */

                //values adjusted for chassis rotation
                yawR = 90 - yawR;
                yawL = 90 + yawL;
                c = 180 - yawR - yawL;
                //find missing angle
            
                //convert values to radians
                double radyawR = yawR * (Math.PI/180);
                double radyawL = yawL *(Math.PI/180);
                double radc = c * (Math.PI/180);

                
                //check if target is centered, offsetleft, or offset right
                if(yawLB < 0 && yawRB > 0){
                    //robot centered between both cameras
                    A = (xDistance / Math.sin(radc)) * Math.sin(radyawR);
                    D = Math.sin(radyawL) * A;
                    Y = D / (Math.tan(radyawL));
                    outputy = Y + xLeftOffset;
                    output = D;

                    SmartDashboard.putNumber("dvalueCentered", D);
                }
                if(yawLB < 0 && yawRB < 0){
                    //robot is offset on the right
                    A = (xDistance / Math.sin(radc)) * Math.sin(Math.abs(radyawL - Math.PI));
                    D = A * Math.sin(radyawR);
                    Y = D / (Math.tan(radyawL));
                    outputy = Y + xLeftOffset;
                    output = D;

                    SmartDashboard.putNumber("dvalueRight", D);

                }
                if (yawLB > 0 && yawRB > 0){
                    //robot is offset on the left
                    A = (xDistance / Math.sin(radc)) * Math.sin(Math.abs(radyawR - Math.PI));
                    D = A * Math.sin(radyawL);
                    Y = D / (Math.tan(radyawR));
                    
                    outputy = (-Y) + xRightOffset;
                    output = D;

                    SmartDashboard.putNumber("dvalueLeft", D);
                }

                SmartDashboard.putNumber("yawl", radyawL);
                SmartDashboard.putNumber("yawR", radyawR);
                SmartDashboard.putNumber("c", radc);
                SmartDashboard.putNumber("outputvalue", D);

                //calculate yaw from ROBOT to target, using difference in yaw from cameras
                //Rotation2d chassisRotation2d= Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
                if(yawLB > yawRB){
                    calcYaw = yawLB - yawRB;
                }
                else if(yawLB < yawRB){
                    calcYaw = yawRB - yawLB;
                }
                //get robot yaw to target
                //Rotation2d yawRobot = Rotation2d.fromDegrees(calcYaw);
                //get target pose
                Optional<Pose3d> Rtargetpose = aprilTagFieldLayout.getTagPose(Rresult.getBestTarget().fiducialId);

                //TODO calculate proper Y values for two cameras, do not use library stuff
                //calculate camera to target translation using robot yaw and distance to target (x axis only)
                //var localpose = PhotonUtils.estimateCameraToTargetTranslation(output, yawRobot);
                //calculate robot pose from the camera to target translation, given localpose, target pose, and gyro angle
                //var pose = PhotonUtils.estimateCameraToTarget(localpose, Rtargetpose.get().toPose2d(), chassisRotation2d);
                Pose2d pose2D = new Pose2d(output, outputy, m_gyro.getRotation2d());
                //calculate true zero of robot given pose and offsets from camera? TODO
                //
                //get yaw to target given pose(s)
                Rotation2d yawtotarget = PhotonUtils.getYawToPose(pose2D, Rtargetpose.get().toPose2d());

                SmartDashboard.putNumber("POSEFx", pose2D.getX());
                SmartDashboard.putNumber("POSEFy", pose2D.getY());
                SmartDashboard.putNumber("POSEFa", pose2D.getRotation().getDegrees());
                SmartDashboard.putNumber("POSEYAW", yawtotarget.getDegrees());
                }
                }
            if(!Lresult.hasTargets() && !Rresult.hasTargets()){
                SmartDashboard.putNumber("POSEFx", 0);
                SmartDashboard.putNumber("POSEFy", 0);
            }
        }
    }



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

    //public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        
       // photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
       // return photonPoseEstimator.update(result);
   // }

}
