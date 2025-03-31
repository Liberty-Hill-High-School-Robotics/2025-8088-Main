package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.visionData;

//https://docs.photonvision.org/en/v2025.1.1/docs/programming/photonlib/getting-target-data.html
//READ THIS ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^


public class Vision extends SubsystemBase {

    //make sure the name in quotes is EXACTLY the same as it is in PV
    PhotonCamera Limelight = new PhotonCamera("Limelight");
    public final Pigeon2 m_gyro = new Pigeon2(CanIDs.GyroID);
    private final Field2d m_field = new Field2d();
    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public Vision(){
        //initalization here
        //none needed ^ 
    }


    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches, etc
        //You can retrieve the latest pipeline result using the PhotonCamera instance."
        // Check if the latest result has any targets.
        //GetLL Values
        if(Limelight.isConnected()){
            var LLResult = Limelight.getLatestResult();
            // Get the current best target.

            if(LLResult.hasTargets()){
                //get all usable data from target:
                //yaw, pitch, area, ID, pose, height
                PhotonTrackedTarget LLbesttarget = LLResult.getBestTarget();
                int LLtargetID = LLbesttarget.getFiducialId();
                Optional<Pose3d> LLtargetpose = aprilTagFieldLayout.getTagPose(LLtargetID);

                SmartDashboard.putNumber("TAGPOSEX", LLtargetpose.get().getX());
                SmartDashboard.putNumber("TAGPOSEY", LLtargetpose.get().getY());
                SmartDashboard.putNumber("TAGPOSEANGLE", LLtargetpose.get().getRotation().getAngle());
                SmartDashboard.putNumber("LTagID", LLbesttarget.objDetectId);

            }

            //get single tag result if it exists
            if(LLResult.hasTargets()){
                Transform3d SinglePose = LLResult.getBestTarget().getBestCameraToTarget();
                Pose2d SinglePose2d = new Pose2d(SinglePose.getX(), SinglePose.getY(), m_gyro.getRotation2d());
                double error = LLResult.getBestTarget().getPoseAmbiguity();
                if(error <= 2){
                    System.out.println("SINGLE TAG ERROR < 2, UPDATING POSE");
                    m_field.setRobotPose(SinglePose2d);
                }
                SmartDashboard.putNumber("ST ERROR", error);
                SmartDashboard.putNumber("ST X", SinglePose.getX());
                SmartDashboard.putNumber("ST Y", SinglePose.getY());
            }
            //override pose if multitag result exists
            if(LLResult.getMultiTagResult().isPresent()){
                Transform3d fieldToCamera = LLResult.getMultiTagResult().get().estimatedPose.best;
                System.out.println("multitag present LL");
                Pose2d multipose2D = new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(), m_gyro.getRotation2d());
                m_field.setRobotPose(multipose2D);
                SmartDashboard.putNumber("MT ERROR", LLResult.getMultiTagResult().get().estimatedPose.bestReprojErr);
                SmartDashboard.putNumber("MT X", multipose2D.getX());
                SmartDashboard.putNumber("MT Y", multipose2D.getY());
            }

            SmartDashboard.putData("Field", m_field);
        }          
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
}
