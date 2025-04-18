// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.validation.SchemaFactory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Align;
import frc.robot.commands.AutoL1;
import frc.robot.commands.AutoL4;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.Coral.Eject;
import frc.robot.commands.Coral.IntakeIn;
import frc.robot.commands.Coral.IntakeOut;
import frc.robot.commands.Coral.PlaceL1;
//Command importsx
import frc.robot.commands.Drive.ZeroHeading;
import frc.robot.commands.Drive.easyAlign;
import frc.robot.commands.Drive.indexGyro;
import frc.robot.commands.Elevator.ElevatorDownDefault;
import frc.robot.commands.Elevator.ElevatorLevel;
import frc.robot.commands.Elevator.ElevatorUp;
//Subsystem imports
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Vision;



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
  public final DriveSubsystem m_drivesubsystem = new DriveSubsystem();


  //create an autonomous chooser
  public final SendableChooser<Command> autoChooser;
  public static double LedState;

  //Create the driver and operator controller. Please use CommandXboxController instead of XboxController
  
  CommandPS5Controller m_driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);


    //The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer(){
    //--------------------Auton stuff-------------------
    //Create namedcommands for PathPlanner paths. Note which names you use, as they have to be exactly the same in PP
    //example here
    //NamedCommands.registerCommand("AutoIntake", new AutoIntakeTimeout(m_intake, m_storage, m_pivot, m_leds));
    NamedCommands.registerCommand("ElevatorL2", new ElevatorLevel(m_elevator, MotorSpeeds.elevatorL2));
    NamedCommands.registerCommand("ElevatorL4", new ElevatorLevel(m_elevator, MotorSpeeds.elevatorL4));
    NamedCommands.registerCommand("ElevatorL4Auto", new AutoL4(m_coral, m_elevator));
    NamedCommands.registerCommand("ElevatorDown", new ElevatorDownDefault(m_elevator));
    NamedCommands.registerCommand("PlaceL1", new AutoL1(m_coral, m_elevator));
    NamedCommands.registerCommand("CoralIn", new IntakeIn(m_coral));

    SmartDashboard.putData("p", new Align(m_drivesubsystem));


    NamedCommands.registerCommand("ElevatorL0", new ElevatorDownDefault(m_elevator));

    NamedCommands.registerCommand("CoralOut", new IntakeOut(m_coral));

    //Pathplanner auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser); 
    SmartDashboard.putNumber("robotcpu", RoboRioDataJNI.getCPUTemp());

    //Put the autons on the chooser and on SmartDashboard
    //example
    //SmartDashboard.putData("AmpPlayoff", new PathPlannerAuto("AmpPlayoff"));
    SmartDashboard.putData("easy", new PathPlannerAuto("easy"));
    SmartDashboard.putData("MOVEHYES", new PathPlannerAuto("MOVE"));



    //------------------------------------------------------------------------------------------
    //----------------------------- Start of SmartDashboard Exports-----------------------------
    //------------------------------------------------------------------------------------------

    //------------------------------------- Command Exports ------------------------------------
    //Algae Exports
    

    //Climber Exports
    SmartDashboard.putData("climber up", new ClimbUp(m_climber));
    SmartDashboard.putData("climber down", new ClimbDown(m_climber));

    //Coral Export
    

    //DriveSubsytem Exports
    SmartDashboard.putNumber("inception", SmartDashboard.getNumber("SDYaw", 0));

    SmartDashboard.putData("GYRO0", new indexGyro(m_drivesubsystem, 0));
    SmartDashboard.putData("GYRO90", new indexGyro(m_drivesubsystem, 90));
    SmartDashboard.putData("GYRO180", new indexGyro(m_drivesubsystem, 180));
    SmartDashboard.putData("GYRO270", new indexGyro(m_drivesubsystem, 270));
    SmartDashboard.putData("GYRO360", new indexGyro(m_drivesubsystem, 360));


    //Elevator Exports
    SmartDashboard.putData("elevator up", new ElevatorUp(m_elevator));
    

    //GroundIntake Exports
    

    //Hopper Exports


    //LEDs Exports
    

    //------------------------------------- Other Exports ------------------------------------
    //SemiAuto Commands


    //Other
    SmartDashboard.putData("ZeroHeading", new ZeroHeading(m_drivesubsystem));

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

   // m_elevator.setDefaultCommand(new ElevatorDownDefault(m_elevator));
    m_elevator.setDefaultCommand(
    new RunCommand(() -> {m_elevator.elevatorStopDefault();
    }, m_elevator));

    m_led.setDefaultCommand(
    new RunCommand(() -> {m_led.DefaultSetPattern();
    }, m_led));


    m_drivesubsystem.setDefaultCommand(
    new RunCommand(() -> {
        var boostRatio = m_driverController.getHID().getR1Button() ? .8 : DriveConstants.basicDriveRatio;
        //check if elevator is high, if so, mutliply robot speed by variable (should halve speed or similar)
        boolean elevatorheight = false;
        double elevatorSlowRatio = 1;

        if(m_elevator.elevatorEncoderGet() < DriveConstants.elevatorHeightSlow){
          elevatorheight = true;
        }
        if(elevatorheight){
          elevatorSlowRatio = DriveConstants.elevatorSpeedRatio;
        }

        m_drivesubsystem.drive(
          //inputs from joystick to drive system
          -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband) * boostRatio * elevatorSlowRatio,
          -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband) * boostRatio * elevatorSlowRatio,
          -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband) * elevatorSlowRatio,
          true); },
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

     //driver joystick

     final Trigger ResetHeading = m_driverController.triangle();
     ResetHeading.onTrue(new ZeroHeading(m_drivesubsystem));

     final Trigger EasyAlignR = m_driverController.circle();
     EasyAlignR.whileTrue(new easyAlign(m_drivesubsystem, true));

     final Trigger EasyAlignL = m_driverController.square();
     EasyAlignL.whileTrue(new easyAlign(m_drivesubsystem, false));

     final Trigger ClimbDown = m_driverController.povDown();
     ClimbDown.whileTrue(new ClimbDown(m_climber));

     final Trigger ClimbUp = m_driverController.povUp();
     ClimbUp.whileTrue(new ClimbUp(m_climber));

     //operator joystick
     //elevator setpoints
     final Trigger ElevatorL4 = m_operatorController.y();
     ElevatorL4.toggleOnTrue(new ElevatorLevel(m_elevator, MotorSpeeds.elevatorL4));

     final Trigger ElevatorL3 = m_operatorController.b();
     ElevatorL3.toggleOnTrue(new ElevatorLevel(m_elevator, MotorSpeeds.elevatorL3));

     final Trigger ElevatorL2 = m_operatorController.x();
     ElevatorL2.toggleOnTrue(new ElevatorLevel(m_elevator, MotorSpeeds.elevatorL2));

     final Trigger ElevatorL1 = m_operatorController.a();
     ElevatorL1.toggleOnTrue(new ElevatorLevel(m_elevator, MotorSpeeds.elevatorL1));

     final Trigger ElevatorDown = m_operatorController.povDown();
     ElevatorDown.whileTrue(new ElevatorDownDefault(m_elevator));

     final Trigger CoralIN = m_operatorController.rightBumper();
     CoralIN.toggleOnTrue(new frc.robot.commands.Coral.IntakeIn(m_coral));

     final Trigger CoralOUT = m_operatorController.leftBumper();
     CoralOUT.whileTrue(new frc.robot.commands.Coral.IntakeOut(m_coral));
     CoralOUT.onFalse(new ElevatorDownDefault(m_elevator));

     final Trigger PlaceL1 = m_operatorController.povLeft();
     PlaceL1.whileTrue(new PlaceL1(m_coral));

     final Trigger Eject = m_operatorController.povRight();
     Eject.whileTrue(new Eject(m_coral));

     //left/right offsets

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