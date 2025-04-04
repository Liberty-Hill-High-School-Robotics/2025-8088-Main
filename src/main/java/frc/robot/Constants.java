// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity. //what the freak does verbosity mean xd
 */

public final class Constants {
  public static final class visionData{
  //xy coords offsets for each april tag
  //17, 18, 19, 20, 21, 22 (blue)
  //6, 7, 8, 9, 10 , 11 (red)




  public static final Transform3d robotToCamLeft = new Transform3d(new Translation3d(0.2413, -0.1905, 0.229), new Rotation3d(0,90,0));
  public static final double leftCamHeight = robotToCamLeft.getZ();
  public static final Rotation3d leftCamAngle = robotToCamLeft.getRotation();
  public static final double leftCamPitch = Math.PI/2;

  public static final Rotation2d robotToCamAngleLeft2D = leftCamAngle.toRotation2d();
  public static final Transform2d robotToCamLeft2D = new Transform2d(robotToCamLeft.getX(), robotToCamLeft.getY(), robotToCamAngleLeft2D);

  

  public static final Transform3d robotToCamRight = new Transform3d(new Translation3d(0.2413, 0.1905, 0.226), new Rotation3d(0,90,0));
  public static final double rightCamHeight = robotToCamRight.getZ();
  public static final Rotation3d rightCamAngle = robotToCamRight.getRotation();
  public static final double rightCamPitch = Math.PI/2;

  public static final Rotation2d robotToCamAngleRight2D = leftCamAngle.toRotation2d();
  public static final Transform2d robotToCamRight2D = new Transform2d(robotToCamRight.getX(), robotToCamRight.getY(), robotToCamAngleRight2D);


  public static final double gyroHeightMeters = 1;



  //newLL constants
  //offsets 
  /*
  * x = 7.5in forwards
  * y = 1in left
  * z = 5 3/4in up
  * r = 0
  * p = 5
  * y = 0
  */
  public static final Transform3d LLCAMERAOFFSET = new Transform3d
  (Units.inchesToMeters(-7.5), Units.inchesToMeters(1), Units.inchesToMeters(5.75), new Rotation3d(0, 11, 0));


  }
  //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  public static final class CanIDs{
    //can IDs
    public static final int BlinkinPWMPort = 0;
    public static final int BeamDIOPort = 0;
    public static final int ClimberBeamDIOPort = 1;
    public static final int GyroID = 9;
    public static final int elevatorMotorID = 10;
    public static final int coralMotorID = 11;
    public static final int climberMotorID = 12;


  }

  public static final class MotorSpeeds{
    public static final double eP = 0.07;
    public static final double eI = 0;
    public static final double eD = 0;

    //down PID constants
    public static final double ePd = 0.004;
    public static final double eId = 0.001;
    public static final double eDd = 0;
    //encoder values for each setpoint, assuming bottom is 0
    //top of elevator is around encoder count = -149.5
    //index to close to zero for faster drop (motors will fall the rest of the way to prevent damage)
    public static final double elevatorL0 = -1;
    public static final double elevatorL1 = -20;
    public static final double elevatorL2 = -43;
    public static final double elevatorL3 = -88;
    public static final double elevatorL4 = -147.75;

    public static final double elevatorSpeed = .55;
    public static final double elevatorSpeedDown = .6;

    public static final double coralSpeed = -.075;
    public static final double coralOutSpeed = -.1;
    public static final double coralBrakeSpeed = -.02;

    public static final double climberUpSpeed = 2500;
    public static final double climberDownSpeed = 3000;

    public static final double climberBottomPos = .5;
    public static final double climberTopPos= -206;

    //climber PID constants
    public static final double cP = .000125;
    public static final double cI = .0000005;
    public static final double cD = 0;

  }

  public static final class ColorConstants{
    //RGB colors
        //purple = 80, 45, 127
        //gold = 255, 200, 46
        //orange = 255, 145, 0
        //blue = 0, 0, 255
        //red = 255, 0, 0
        //green = 0, 255, 0
        public static int[] purple = new int[]{80, 3, 143}; 
        public static int[] gold   = new int[]{255, 145, 0}; 
        public static int[] orange = new int[]{255, 145, 0}; 
        public static int[] blue   = new int[]{0, 0, 255}; 
        public static int[] red    = new int[]{255, 0, 0}; 
        public static int[] green  = new int[]{0, 255, 0};
  }


  public static final class DriveConstants {
    public static final double basicDriveRatio = 0.6; //base drive ratio (.8 of max speed)
    public static final double elevatorSpeedRatio = 0.25; //ratio to slow down to if elevator is high enough
    public static final double elevatorHeightSlow = -40; //height to slow down after, in encoder counts (must be negative)
    public static final double elevatorHeightOff = -5; //height to let elevator drop after (so we dont slam down)


    public static final double leftXOffset = .069;
    public static final double rightXOffset = -.18; //value good dont change unless really needed
    public static final double yOffset = .5;
    public static final double rOffset = 0;

    //TUNE THESE!!!
    //forward
    public static final double xP = .765;
    public static final double xI = 0;
    public static final double xD = 0;
    //sideways
    public static final double yP = 3;
    public static final double yI = 0.1;
    public static final double yD = 0;
    public static final double aP = 0.05;
    public static final double aI = 0;
    public static final double aD = 0;


    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        
        //same thing as above but used in the KINEMATICS class
    public static final Translation2d[] Module_Info = {
          new Translation2d(kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
          new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        };
    

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFLDrivingCAN = 1;
    public static final int kFRDrivingCAN = 2;
    public static final int kBLDrivingCAN = 3;
    public static final int kBRDrivingCAN = 4;

    public static final int kFLTurningCAN = 5;
    public static final int kFRTurningCAN = 6;
    public static final int kBLTurningCAN = 7;
    public static final int kBRTurningCAN = 8;

    public static final boolean kGyroReversed = false;
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(Module_Info);
  }



  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }



  public static final class OIConstants {
    //controller ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.2;
  }



  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }


  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
