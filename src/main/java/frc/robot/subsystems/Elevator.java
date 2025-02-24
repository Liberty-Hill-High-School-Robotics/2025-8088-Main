package frc.robot.subsystems;


import javax.naming.spi.NamingManager;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;



public class Elevator extends SubsystemBase {

    PIDController ElevatorPID = new PIDController(MotorSpeeds.eP, MotorSpeeds.eI, MotorSpeeds.eD);

    //motors & variables here, define them and create any PIDs needed
    private SparkMax elevatorSparkMax;
    private RelativeEncoder elevatorRelativeEncoder;
    private SparkLimitSwitch elevatorBottomLimit;
    private SparkLimitSwitch elevatorTopLimit;

    public Elevator(){
        //config motor settings here
        //config motor settings here
        elevatorSparkMax = new SparkMax(CanIDs.elevatorMotorID, MotorType.kBrushless);
        elevatorBottomLimit = elevatorSparkMax.getReverseLimitSwitch();
        //elevatorTopLimit = elevatorSparkMax.getForwardLimitSwitch();
        
        //elevatorRelativeEncoder.
        elevatorRelativeEncoder = elevatorSparkMax.getEncoder();
    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
        SmartDashboard.putBoolean("elevatorbottom", elevatorAtBottomLimit());
        SmartDashboard.putNumber("elevatorencoder", elevatorEncoderGet());

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

    public void elevatorUp(){
        elevatorSparkMax.set(MotorSpeeds.elevatorSpeed);
    }

    public void elevatorDown(){
        elevatorSparkMax.set(-MotorSpeeds.elevatorSpeed);
    }

    //elevator setpoints
    public void elevatorL(double setpoint){
        double calc = ElevatorPID.calculate(elevatorRelativeEncoder.getPosition(), setpoint);
        elevatorSparkMax.set(calc);
    }

    public void elevatorStop(){
        elevatorSparkMax.stopMotor();
    }

    public boolean elevatorAtBottomLimit(){
        return elevatorBottomLimit.isPressed();
    }

    public boolean elevatorAtTopLimit(){
        //return elevatorTopLimit.isPressed();
        return false; 
    }

    public double elevatorEncoderGet(){
        return elevatorRelativeEncoder.getPosition();
    }
}
