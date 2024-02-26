// 

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;

  private RelativeEncoder encoder, encoder1;
  private SparkPIDController controller, controller1;

  private boolean m_speakerShootRunning;
  private boolean m_ampShootRunning;
  

  /** Creates a new LauncherSubsystem. */
  public ShooterSubsystem() {
    // create two new SPARK MAXs and configure the
    m_leftMotor =
        new CANSparkMax(Constants.ShooterConstants.leftShooterMotorId, CANSparkLowLevel.MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();

    m_leftMotor.setInverted(true);
    m_leftMotor.setSmartCurrentLimit(80);
    m_leftMotor.setIdleMode(IdleMode.kCoast);

    m_leftMotor.burnFlash();

    m_rightMotor =
        new CANSparkMax(Constants.ShooterConstants.rightShooterMotorId, CANSparkLowLevel.MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setInverted(false);
    m_rightMotor.setSmartCurrentLimit(80);
    m_rightMotor.setIdleMode(IdleMode.kCoast);
    //m_rightMotor.follow(m_leftMotor, true);
    m_rightMotor.burnFlash();

    encoder = m_leftMotor.getEncoder();
    controller = m_leftMotor.getPIDController();
    controller.setFeedbackDevice(encoder);
    controller.setOutputRange(-1, 1);
    controller.setSmartMotionMaxVelocity(6000, 0);
    controller.setSmartMotionMaxAccel(6000, 0);
    controller.setP(0);
    controller.setI(0);
    controller.setD(0);
    controller.setFF(0.05);

    encoder1 = m_rightMotor.getEncoder();
    controller1 = m_rightMotor.getPIDController();
    controller1.setFeedbackDevice(encoder1);
    controller1.setOutputRange(-1, 1);
    controller1.setSmartMotionMaxVelocity(6000, 0);
    controller1.setSmartMotionMaxAccel(6000, 0);
    controller1.setP(0);
    controller1.setI(0);
    controller1.setD(0);
    controller1.setFF(0.05);

    m_speakerShootRunning = false;
    m_ampShootRunning = false;
    
  }

  public boolean getAtSpeed(double speed){
    return encoder.getVelocity() >= speed+100 || encoder.getVelocity() >= speed-100;
  }

 
  public void speakerShoot() {
    m_speakerShootRunning = true;
  }

  public void ampShoot() {
    m_ampShootRunning = true;
  }

  public void stopShooter() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  public void runShooter(double speed){
    controller.setReference(speed, ControlType.kVelocity);
    controller1.setReference(speed, ControlType.kVelocity);
    //m_rightMotor.set(.4);
    System.out.println("in run shooter method");
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
    /*if (m_ampShootRunning) {
      m_leftMotor.set(Constants.ShooterConstants.kAmpPower);
      m_rightMotor.set(-Constants.ShooterConstants.kAmpPower);
    }
    if(m_speakerShootRunning){
      m_leftMotor.set(Constants.ShooterConstants.kSpeakerPower);
      m_rightMotor.set(Constants.ShooterConstants.kSpeakerPower);
    }
    else {
      m_leftMotor.set(0.0);
      m_rightMotor.set(0.0);
    }*/

    SmartDashboard.putNumber("RPM", encoder1.getVelocity());
  }
}