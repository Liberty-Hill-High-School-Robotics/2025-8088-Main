package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;
import frc.robot.Constants.MotorSpeeds;



public class Climber extends SubsystemBase {

    private SparkMax climberSparkMax;
    private SparkClosedLoopController climberPID;
    

    public Climber(){
        //config motor settings here
        climberSparkMax = new SparkMax(CanIDs.climberMotorID, MotorType.kBrushless);
        climberPID = climberSparkMax.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(MotorSpeeds.cP, MotorSpeeds.cI, MotorSpeeds.cD, 1/473); //reciprocal of the motor's velocity constant
        climberSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  

    @Override
    public void periodic() {
        //This method will be called once per scheduler run
        //Put smartdashboard stuff, check for limit switches
        SmartDashboard.putNumber("Climber Velocity", climberSparkMax.getEncoder().getVelocity());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
        //Mostly used for debug and such
    }

    public void climberUp() {
        climberPID.setReference(MotorSpeeds.climberUpSpeed, ControlType.kVelocity);
    }
    public void climberDown() {
        climberPID.setReference(-MotorSpeeds.climberDownSpeed, ControlType.kVelocity);
    }
    public void climberStop() {
        climberSparkMax.set(0);
    }

}
