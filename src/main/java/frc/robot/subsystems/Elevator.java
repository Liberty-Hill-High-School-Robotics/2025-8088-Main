package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;



public class Elevator extends SubsystemBase {

    //motors & variables here, define them and create any PIDs needed
    private SparkMax elevatorSparkMax;
    private SparkMax elevatorSparkMax2;
    private RelativeEncoder elevatorRelativeEncoder;
    private RelativeEncoder elevatorRelativeEncoder2;
    private SparkLimitSwitch elevatorBottomLimit;
    private SparkLimitSwitch elevatorTopLimit;

    public Elevator(){
        //config motor settings here
        //config motor settings here
        elevatorSparkMax = new SparkMax(CanIDs.elevatorMotorID, MotorType.kBrushless);
        elevatorBottomLimit = elevatorSparkMax.getReverseLimitSwitch();

        elevatorSparkMax2 = new SparkMax(CanIDs.elevatorMotor2ID, MotorType.kBrushless);
        elevatorTopLimit = elevatorSparkMax2.getForwardLimitSwitch();
        
        //elevatorRelativeEncoder.
        elevatorRelativeEncoder = elevatorSparkMax.getEncoder();
        elevatorRelativeEncoder2 = elevatorSparkMax2.getEncoder();
    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
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
        elevatorSparkMax.set(MotorSpeeds.elevatorSpeed);
    }

    public void elevatorDown(){
        elevatorSparkMax.set(-MotorSpeeds.elevatorSpeed);
        elevatorSparkMax.set(-MotorSpeeds.elevatorSpeed);
    }

    public void elevatorStop(){
        elevatorSparkMax.stopMotor();
        elevatorSparkMax.stopMotor();
    }

    public boolean elevatorAtBottomLimit(){
        return elevatorBottomLimit.isPressed();
    }

    public boolean elevatorAtTopLimit(){
        return elevatorTopLimit.isPressed();
    }
}
