// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import com.pathplanner.lib.auto.AutoBuilder;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.hardware.*;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  RobotConfig driveConfig;

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

  public CommandXboxController m_driverControllerLocal = new CommandXboxController(OIConstants.kDriverControllerPort);
  //add a turning PID to manually control the turning of the robot, and translation pid
  PIDController TranslationPID = new PIDController(DriveConstants.xP, DriveConstants.xI, DriveConstants.xD);

  TrajectoryConfig PathConfig = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAngularSpeed);
  
  //create holonomicdrivecontroller for path following
  HolonomicDriveController controller = new HolonomicDriveController(
  new PIDController(DriveConstants.xP, DriveConstants.xI, DriveConstants.xD), 
  new PIDController(DriveConstants.yP, DriveConstants.yI, DriveConstants.yD),
  new ProfiledPIDController(1, 0, 0,
    new TrapezoidProfile.Constraints(DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxSpeedMetersPerSecond)));


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
        (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyrovalue", getHeading());
    
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }
        //,visionpose here
        //NEED TO GET POSE2D FROM POSE ESTIMATOR IN VISION SUBSYSTEM
        );
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


  public void leftrightPIDcontrol(double current){
    double calc = TranslationPID.calculate(SmartDashboard.getNumber("SDYaw", 0), 0);
    double calcD = TranslationPID.calculate(SmartDashboard.getNumber("SDDistance", 0), 15);
    //double calcR = turningPID.calculate(SmartDashboard.getNumber("SDYaw", 0), 0);

    var speeds = new ChassisSpeeds(calcD, calc, 0);
    //apply these speeds
    setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(speeds));
  }


  public void TESTRUN(){
    double distance = SmartDashboard.getNumber("distanceValue", 0);
    double rotation = SmartDashboard.getNumber("angledegrese", 180);
    double calc = TranslationPID.calculate(distance, 3.5);
    double calcR = TranslationPID.calculate(m_gyro.getYaw().getValueAsDouble(), rotation);


    var speeds = new ChassisSpeeds((-calc * DriveConstants.kMaxAngularSpeed),
                                   (-MathUtil.applyDeadband(m_driverControllerLocal.getLeftX(), OIConstants.kDriveDeadband) * DriveConstants.kMaxAngularSpeed),
                                   (calcR));
    //apply swerve module states

    setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(speeds));
  }


  public void FullPIDControl(){
    Pose2d end = new Pose2d(
    SmartDashboard.getNumber("poseLXT", 0), 
    SmartDashboard.getNumber("poseLYT", 0), 
    Rotation2d.fromDegrees(SmartDashboard.getNumber("poseLRT", 0)));

    Pose2d robotpose = new Pose2d(
    SmartDashboard.getNumber("poseLX", 0), 
    SmartDashboard.getNumber("poseLY", 0), 
    Rotation2d.fromDegrees(SmartDashboard.getNumber("poseLR", 0)));

    List<Pose2d> poselist = new ArrayList<>();
    poselist.add(end);
    poselist.add(robotpose);

    PathConfig.setEndVelocity(0);
    PathConfig.setStartVelocity(0);
    var trajectory = TrajectoryGenerator.generateTrajectory(poselist, PathConfig);
    ChassisSpeeds controlledSpeeds = controller.calculate(robotpose, trajectory.sample(trajectory.getTotalTimeSeconds()), robotpose.getRotation());

    //apply these speeds
    setModuleStates(DriveConstants.KINEMATICS.toSwerveModuleStates(controlledSpeeds));
  }
}