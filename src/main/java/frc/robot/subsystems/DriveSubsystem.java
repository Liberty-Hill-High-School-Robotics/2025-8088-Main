// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

public class DriveSubsystem extends SubsystemBase {
  RobotConfig driveConfig;
  boolean rightoffset = false;
  int timeStep = 0;
  double index = 0;


  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFLDrivingCAN,
      DriveConstants.kFLTurningCAN,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFRDrivingCAN,
      DriveConstants.kFRTurningCAN,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kBLDrivingCAN,
      DriveConstants.kBLTurningCAN,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kBRDrivingCAN,
      DriveConstants.kBRTurningCAN,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  public final Pigeon2 m_gyro = new Pigeon2(CanIDs.GyroID);

  public CommandPS5Controller m_driverControllerLocal = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  public CommandXboxController m_operatorControllerLocal = new CommandXboxController(OIConstants.kDriverControllerPort);

  private final Field2d m_field = new Field2d();


  //add a turning PID to manually control the turning of the robot, and translation pid
  PIDController TranslationPID = new PIDController(DriveConstants.yP, DriveConstants.yI, DriveConstants.yD);
  PIDController SidewaysPID = new PIDController(DriveConstants.xP, DriveConstants.xI, DriveConstants.xD);

  PIDController RotationPID = new PIDController(DriveConstants.aP, DriveConstants.aI, DriveConstants.aD);


  TrajectoryConfig PathConfig = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAngularSpeed);
  
  //create holonomicdrivecontroller for path following
  HolonomicDriveController controller = new HolonomicDriveController(
  new PIDController(0.01, 0, 0), 
  new PIDController(0.01, 0, 0),
  new ProfiledPIDController(0.01, 0, 0,
    new TrapezoidProfile.Constraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAngularSpeed)));


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    try{
      driveConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

      //autobuilder needs to be configured last, add anything before this
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //this::setChassisSpeeds,
        (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
        ),
        driveConfig, // The robot configuration
        () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
);
  }

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("pose", Pose2d.struct).publish();

  @Override
  public void periodic() {
    double VISIONPOSEx = SmartDashboard.getNumber("XPOSEF", m_odometry.getPoseMeters().getX());
    double VISIONPOSEy = SmartDashboard.getNumber("YPOSEF", m_odometry.getPoseMeters().getY());

    boolean LMULTI = SmartDashboard.getBoolean("LMULTITAG", false);
    boolean RMULTI = SmartDashboard.getBoolean("RMULTITAG", false);

    Pose2d visionpose = new Pose2d(VISIONPOSEx, VISIONPOSEy, m_gyro.getRotation2d());

      m_odometry.update(m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
      );
      
      if(LMULTI || RMULTI){
        m_odometry.resetPose(visionpose);
      }

    publisher.set(m_odometry.getPoseMeters());

    SmartDashboard.putData("FieldDRIVE", m_field);
    m_field.setRobotPose(m_odometry.getPoseMeters());


    SmartDashboard.putNumber("gyrovalue", getHeading());
    SmartDashboard.putNumber("drivetrainposeX", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("drivetrainposeY", m_odometry.getPoseMeters().getY());
    //update odometry if vision exists


    /*
    // Update the odometry in the periodior block
    m_odometry.update(m_gyro.getRotation2d().times(-1),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
      //,visionpose here
      //NEED TO GET POSE2D FROM POSE ESTIMATOR IN VISION SUBSYSTEM
      );
      */

    
    
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   *    
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return (m_gyro.getAngularVelocityZWorld()).getValueAsDouble();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.KINEMATICS.toChassisSpeeds(
      // supplier for chassisSpeed, order of motors need to be the same as the consumer of ChassisSpeed
      m_frontLeft.getState(), 
      m_rearLeft.getState(),
      m_frontRight.getState(),
      m_rearRight.getState()
      );
  }

  //see drive constants for details
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
      DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds));
  }

  public boolean rightOffset(boolean value) {
    rightoffset = value;
    return rightoffset;
  }

  public void indexZero(){
    index = 0;
  }
  

  public List<Trajectory.State> TrajGenerate(){
    //instansiate final pose
    Pose2d FINALPOSE;
    //create pose of the robot
    Pose2d robotpose = m_odometry.getPoseMeters();

    PathConfig.setEndVelocity(0); //should always be zero
    PathConfig.setStartVelocity(0); //tbd (messing with values rn)


    //set finalpose
    if(rightoffset){
      FINALPOSE = new Pose2d(DriveConstants.yOffset, DriveConstants.rightXOffset, 
      Rotation2d.fromDegrees(SmartDashboard.getNumber("poseLRT", 0) + DriveConstants.rOffset));
    }
    else{
      FINALPOSE = new Pose2d(DriveConstants.yOffset, DriveConstants.leftXOffset, 
      Rotation2d.fromDegrees(SmartDashboard.getNumber("poseLRT", 0) + DriveConstants.rOffset));
    }

    //TODO override
    FINALPOSE = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

    //create and add stuff to a pose list
    List<Pose2d> poselist = new ArrayList<>();
    poselist.add(robotpose); //start pose
    poselist.add(FINALPOSE); //end pose

    var trajectory = TrajectoryGenerator.generateTrajectory(poselist, PathConfig); //generate traj.
    timeStep = 0;

    //get all states along the spline
    List<Trajectory.State> SplineStates = trajectory.getStates();

    //return states along the spline
    return SplineStates;
  }


  public void FullPIDControl(List<Trajectory.State> splines){
    try{
      //set robot pose
      Pose2d robotPose = m_odometry.getPoseMeters();
      //get ending states
      Trajectory.State endingState = splines.get(splines.size());
      Rotation2d endingRotation = endingState.poseMeters.getRotation();

      //get next state
      Trajectory.State nextState = splines.get(timeStep);

      //calculate output speeds at timeStep
      ChassisSpeeds controlledSpeeds = controller.calculate(robotPose, nextState, endingRotation);
    
      setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(controlledSpeeds));

      //increase timestep for next run of path following
      timeStep++;

    }
    catch(Exception e){
      System.out.println("GENERAL ERROR PATH FOLLOWING; INFORM PROGRAMMER");
    }
  }

  //update odometry using vision pose IF it exists
  public void UpdateODO(){
    //get vision pose, values = 0 if DNE
    double VisionX = SmartDashboard.getNumber("POSEFx", 0);
    double VisionY = SmartDashboard.getNumber("POSEFy", 0);
    //create pose2d object using vision pose
    Pose2d VisionPose = new Pose2d(VisionX, VisionY, m_gyro.getRotation2d());

    //check if pose exists
    if(VisionX != 0 && VisionY != 0){
      //update m_odo using gyro, encoders, and vision pose
      m_odometry.resetPose(VisionPose);
    }
  }


  //get proper yaw values to tag, tune translation pids, and make sure it works without close ups

  
  //PID controls for x and y values of the cameras!
  public void SimpleAlign(boolean Right){
    //easy right align it works, 
    System.out.println("ALIGNING");
    //TODO make this not run way to the right if tag not seen 

    //get tag angle
    double Langle = SmartDashboard.getNumber("PoseLRT", m_gyro.getYaw().getValueAsDouble());
    double Rangle = SmartDashboard.getNumber("PoseRRT", m_gyro.getYaw().getValueAsDouble());
    double asetpoint = m_gyro.getYaw().getValueAsDouble(); //init as default value, is changed later
    int LtagID = (int )SmartDashboard.getNumber("LTagID", 0); //get tag ID to see if it exists
    int RtagID = (int )SmartDashboard.getNumber("RTagID", 0); //get tag ID to see if it exists

    //create setpoints for x and y TODO: send to constants.java
    double xLsetpoint = DriveConstants.leftXOffset;
    double xRsetpoint = DriveConstants.rightXOffset;
    double ysetpoint = 0;

    //get x and y from smartdashboard (and rotation of target)
    //TODO either update drivetrain pose to get close tag tracking or make dummy default values?
    //TODO maybe add a timeout ^
    int tagId = LtagID;
    asetpoint = Rangle;
    if(Right){
      asetpoint = Langle;
      tagId = RtagID;
    }
    double VisionY = SmartDashboard.getNumber("POSEFy", xRsetpoint);
    double VisionX = SmartDashboard.getNumber("POSEFx", 0);
    double VisionA = m_gyro.getYaw().getValueAsDouble();
    

    //calculate PID values for both!
    double calcY = TranslationPID.calculate(VisionY, xLsetpoint);
    if(Right){
      calcY = TranslationPID.calculate(VisionY, xRsetpoint);
    }
    double calcX = SidewaysPID.calculate(VisionX, ysetpoint);
    double calcA = RotationPID.calculate(VisionA, asetpoint);
    
    if (true) { //TODO change this to smthing that detects if camera has tag
      //create a chassisspeed class given these values
      ChassisSpeeds controlledSpeeds = new ChassisSpeeds(calcX, calcY, calcA);
      //command drivetrain with x, y and omega
      setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(controlledSpeeds));
    }
  }

}