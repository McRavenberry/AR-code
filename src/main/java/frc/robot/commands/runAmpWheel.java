package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class runAmpWheel extends Command {
  /** Creates a new ArmScore. */
  private AmperSubsystem s_Amper;
  private ShooterSubsystem s_Shooter;
  private IntakeSubsystem s_Intake;
  

  public runAmpWheel(AmperSubsystem s_Amper, ShooterSubsystem s_Shooter,IntakeSubsystem s_Intake) {
    this.s_Amper = s_Amper;
    this.s_Shooter = s_Shooter;
    this.s_Intake = s_Intake;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Amper);
    addRequirements(s_Shooter);
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(s_Amper.atPos(6500)){
      s_Amper.setSpeed(0.9);
      s_Shooter.runShooter(500);

      if(s_Shooter.getAtSpeed(0.2)){
        s_Intake.setMotor(-0.9);
      }
  }

    System.out.println("RUN AMP");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.runShooter(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
