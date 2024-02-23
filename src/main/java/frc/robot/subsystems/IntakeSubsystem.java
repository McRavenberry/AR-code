package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
//import com.ctre.phoenix.motorcontrol.SensorCollection;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IntakeSubsystem extends SubsystemBase{
    private TalonSRX intakeWheel;
    private TalonFX intakeArm;
    private PositionVoltage positionVoltage;

    public IntakeSubsystem(){
        intakeWheel = new TalonSRX(Constants.IntakeConstants.intakeWheelMotorID);
        intakeArm = new TalonFX(Constants.IntakeConstants.intakeArmMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        intakeArm.getConfigurator().apply(config);

        intakeArm.setPosition(0);

        positionVoltage = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
    }

    public void setMotor(double speed){
        intakeWheel.set(TalonSRXControlMode.PercentOutput , speed);
    }

    public void setArm(double pos){
        intakeArm.setControl(positionVoltage.withPosition(pos));
    }
  
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Intake Arm Encoder: ", intakeArm.getPosition().getValueAsDouble());
    }
}
