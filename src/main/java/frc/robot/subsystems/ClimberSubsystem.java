package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax climberMotor;
    private final RelativeEncoder climbEncoder;
    //private final SparkPIDController climbPIDController;


    public ClimberSubsystem(){
        climberMotor = new CANSparkMax(Constants.ClimberConstants.climberMotorId, MotorType.kBrushless);
        climberMotor.setInverted(false);
        climbEncoder = climberMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor,42);

    }
    

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("Climber Encoder Value: ",climbEncoder );
    }

    public void setMotors(double speed){
        climberMotor.set(speed); 
        
    }
    
    
    
}

