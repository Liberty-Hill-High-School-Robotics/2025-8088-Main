package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;



public class Coral extends SubsystemBase {
    private SparkMax coralMotor;
    public static DigitalInput throughSensor = new DigitalInput(CanIDs.BeamDIOPort);




    public Coral(){
        //config motor settings here
        coralMotor = new SparkMax(CanIDs.coralMotorID, MotorType.kBrushless);        
    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
        SmartDashboard.putBoolean("beam", throughSensor.get());
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

    public void coralOut(){
        coralMotor.set(-MotorSpeeds.coralOutSpeed);
    }

    public void coralOutSlow(){
        coralMotor.set(0.1);
    }

    public void coralOutReverse(){
        coralMotor.set(MotorSpeeds.coralSpeed/1.5);
    }

    public void coralIn(){
        coralMotor.set(-MotorSpeeds.coralSpeed);
    }

    public void coralStop() {
        coralMotor.stopMotor();
    }

    public void coralBrake(){
        coralMotor.set(MotorSpeeds.coralBrakeSpeed);
    }

    public boolean coralGetBeam(){
        return !throughSensor.get();
    }

}
