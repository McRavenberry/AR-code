package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax leftShooterMotor;
  private CANSparkMax rightShooterMotor;

  /** Creates a new LauncherSubsystem. */
  public ShooterSubsystem() {
    // create two new SPARK MAXs and configure them
    leftShooterMotor =
        new CANSparkMax(Constants.ShooterConstants.leftShooterMotorId, CANSparkLowLevel.MotorType.kBrushless);
    leftShooterMotor.setInverted(true);
    leftShooterMotor.setSmartCurrentLimit(50);
    leftShooterMotor.setIdleMode(IdleMode.kBrake);

    leftShooterMotor.burnFlash();

    rightShooterMotor =
        new CANSparkMax(Constants.ShooterConstants.rightShooterMotorId, CANSparkLowLevel.MotorType.kBrushless);
    rightShooterMotor.setInverted(true);
    rightShooterMotor.setSmartCurrentLimit(50);
    rightShooterMotor.setIdleMode(IdleMode.kBrake);

    rightShooterMotor.burnFlash();


    leftShooterMotor.follow(rightShooterMotor, true);

  }

  public void setSpeedShoot(){
    leftShooterMotor.set(.9);
  }
  public void setSpeedAmp(){
    leftShooterMotor.set(.2);
  }
  public void setSpeedAmpEnd(){
    leftShooterMotor.stopMotor();
  }

  @Override
  public void periodic() { // this method will be called once per scheduler run
    // set the launcher motor powers based on whether the launcher is on or not
  }
}