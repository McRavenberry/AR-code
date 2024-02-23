package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.TalongSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AmperSubsystem extends SubsystemBase {
    private TalonSRX ampArm; 
    private CANSparkMax ampWheel;

    public AmperSubsystem(){
        ampArm = new TalonSRX(Constants.AmperConstants.ampArmMotorID);
        ampWheel = new CANSparkMax(Constants.AmperConstants.ampWheelMotorId, MotorType.kBrushless);
        ampArm.setInverted(false);
        ampWheel.setInverted(false);
        ampArm.setSelectedSensorPosition(0.0);

        ampArm.configFactoryDefault();
        ampArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        ampArm.setSensorPhase(false);
        ampArm.setInverted(true);
        ampArm.configNominalOutputForward(0);
        ampArm.configNominalOutputReverse(0);
        ampArm.configPeakOutputForward(1);
        ampArm.configPeakOutputReverse(-1);
        ampArm.configMotionAcceleration(100000000);
        ampArm.configMotionCruiseVelocity(1000000);

        ampArm.config_kF(0, 0.04);
        ampArm.config_kP(0, 0.75);
    }

    public void setSpeed(double speed){
        ampWheel.set(speed);
    }

    public void setArm(double pos){
        ampArm.set(TalonSRXControlMode.MotionMagic, pos);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Encoder", ampArm.getSelectedSensorPosition());
    }
    
}
