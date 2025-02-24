// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
unused imports
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;
import com.pathplanner.lib.commands.PathPlannerAuto;
//import com.pathplanner.lib.commands.PathPlannerAuto;
*/

package frc.robot;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.auto.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;


//Subsystem imports
import frc.robot.subsystems.*;
//Command imports
import frc.robot.commands.Drive.*;
import frc.robot.commands.Vision_LEDS.SetLEDPattern;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  public final Elevator m_elevator = new Elevator();
  public final Coral m_coral = new Coral();
  public final Climber m_climber = new Climber();
  public final Algae m_algae = new Algae();
  public final LED m_led = new LED();
  public final Vision m_vision = new Vision();
  private final DriveSubsystem m_drivesubsystem = new DriveSubsystem();


  //create an autonomous chooser
  public final SendableChooser<Command> autoChooser;

  //Create the driver and operator controller. Please use CommandXboxController instead of XboxController
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);


    //The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer(){
    //--------------------Auton stuff-------------------
    //Create namedcommands for PathPlanner paths. Note which names you use, as they have to be exactly the same in PP
    //example here
    //NamedCommands.registerCommand("AutoIntake", new AutoIntakeTimeout(m_intake, m_storage, m_pivot, m_leds));

    //Pathplanner auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser); 
    SmartDashboard.putNumber("robotcpu", RoboRioDataJNI.getCPUTemp());

    //Put the autons on the chooser and on SmartDashboard
    //example
    //SmartDashboard.putData("AmpPlayoff", new PathPlannerAuto("AmpPlayoff"));

    //------------------------------------------------------------------------------------------
    //----------------------------- Start of SmartDashboard Exports-----------------------------
    //------------------------------------------------------------------------------------------

    //------------------------------------- Command Exports ------------------------------------
    //Algae Exports
    

    //Climber Exports
    

    //Coral Exports
    

    //DriveSubsytem Exports
    SmartDashboard.putData("TESTRUN", new TESTRUN(m_drivesubsystem));
    SmartDashboard.putNumber("inception", SmartDashboard.getNumber("SDYaw", 0));


    //Elevator Exports
    

    //GroundIntake Exports
    

    //Hopper Exports


    //LEDs Exports
    SmartDashboard.putData("SETLED", new SetLEDPattern(m_led, .7));

    //------------------------------------- Other Exports ------------------------------------
    //SemiAuto Commands


    //Other


    //------------------------------------------------------------------------------------------
    //------------------------------ End of SmartDashboard Exports------------------------------
    //------------------------------------------------------------------------------------------


    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    //outputs are multiplied by a boolean controlled by a button on the driver controller
    //makes it slower or faster depending on output of button
    m_drivesubsystem.setDefaultCommand(
    new RunCommand(() -> {
        var boostRatio = m_driverController.getHID().getLeftBumperButton() ? 1 : .8;
        m_drivesubsystem.drive(
          //inputs from joystick to drive system
          -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband) * boostRatio,
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband) * boostRatio,
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
          false); },
          m_drivesubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //add button bindings here
    /*
     * ex:
     * final Trigger commandname = m_drivercontroller.button();
     * commandname.toggleontrue(new commandname(m_subsystem(s)));
     */

     final Trigger AlignXButton = m_driverController.b();
     AlignXButton.whileTrue(new LeftRightPID(m_drivesubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected(); 
    //this allows for pathplanner to use auton stuff, rev provides an example but pathplanner is better
  }
}
