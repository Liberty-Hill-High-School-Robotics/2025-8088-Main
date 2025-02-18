package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

//all imports here
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIDs;



public class Elevator extends SubsystemBase {

    //motors & variables here, define them and create any PIDs needed
    private CANSparkMax elevatorSparkMax;
    private CANSparkMax elevatorSparkMax2;
    private RelativeEncoder elevatorRelativeEncoder;
    private RelativeEncoder elevatorRelativeEncoder2;
    private SparkLimitSwitch elevatorReverseLimit;
    private SparkLimitSwitch elevatorForwardLimit;

    public Elevator(){
        //config motor settings here
        //config motor settings here
        elevatorSparkMax = new CANSparkMax(CanIDs.elevatorMotorID, MotorType.kBrushless);
        elevatorSparkMax.restoreFactoryDefaults();
        elevatorSparkMax.setInverted(true);
        elevatorSparkMax.setIdleMode(IdleMode.kBrake);
        elevatorSparkMax.setSmartCurrentLimit(40);

        elevatorReverseLimit = elevatorSparkMax.getReverseLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen);

        elevatorSparkMax2 = new CANSparkMax(CanIDs.elevatorMotor2ID, MotorType.kBrushless);
        elevatorSparkMax2.restoreFactoryDefaults();
        elevatorSparkMax2.setInverted(true);
        elevatorSparkMax2.setIdleMode(IdleMode.kBrake);
        elevatorSparkMax2.setSmartCurrentLimit(40);

        elevatorForwardLimit = elevatorSparkMax2.getForwardLimitSwitch(com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen);
        
        //elevatorRelativeEncoder.
        elevatorRelativeEncoder = elevatorSparkMax.getEncoder();
        elevatorRelativeEncoder2 = elevatorSparkMax2.getEncoder();

        /*
        ex:
        barRotatorSparkMax = new CANSparkMax(CanIDs.barRotatorID, MotorType.kBrushless);
        //barRotatorSparkMax.restoreFactoryDefaults();
        barRotatorSparkMax.setInverted(true);
        barRotatorSparkMax.setIdleMode(IdleMode.kBrake);
        barRotatorSparkMax.setSmartCurrentLimit(40);

        barReverseLimitSwitch = barRotatorSparkMax.getReverseLimitSwitch(Type.kNormallyOpen);

        barRotatorSparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
        barRotatorSparkMax.setSoftLimit(SoftLimitDirection.kForward, BarConstants.fLimit);

        barRotatorRelativeEncoder = barRotatorSparkMax.getEncoder();
        */
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

    public void ___(){
        //motor.set(PID.calculate(position, setpoint));
        //motor.set(number);
        
    }

}
