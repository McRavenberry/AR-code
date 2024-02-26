package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private final CANSparkMax climberMotor;
    private final RelativeEncoder climbEncoder;
    private final SparkPIDController climbPIDController;


    public ClimberSubsystem(){
        climberMotor = new CANSparkMax(Constants.ClimberConstants.climberMotorId, MotorType.kBrushless);
        climberMotor.restoreFactoryDefaults();
        climberMotor.setInverted(true);
        climberMotor.setSmartCurrentLimit(50); 
        climberMotor.setIdleMode(IdleMode.kBrake);
        climbEncoder = climberMotor.getEncoder();
        climbPIDController = climberMotor.getPIDController();
        climbPIDController.setOutputRange(-1, 1);
        climbPIDController.setSmartMotionMaxVelocity(30000, 0);
        climbPIDController.setSmartMotionMaxAccel(25000, 0);
        climbPIDController.setP(0);
        climbPIDController.setI(0);
        climbPIDController.setD(0);
        climbPIDController.setFF(0.00006);
        climbPIDController.setFeedbackDevice(climbEncoder);

        climbEncoder.setPosition(0.0);
    }

    public void setMotors(double pos){
        climbPIDController.setReference(pos, ControlType.kSmartMotion);
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber pos", climbEncoder.getPosition());
    }
}

