package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class runShooter extends Command {
  /** Creates a new ArmScore. */
    private ShooterSubsystem s_Shooter;
    private Boolean protect = false;
    private IntakeSubsystem s_Intake;

  public runShooter(IntakeSubsystem s_Intake, ShooterSubsystem s_Shooter, Boolean protect) {
    this.s_Intake = s_Intake;
    this.s_Shooter = s_Shooter;
    this.protect = protect;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(protect){
      s_Shooter.setSpeedAmpEnd();
    }
    else{
      s_Shooter.setSpeedShoot();
      s_Intake.setMotor(-0.2);
    }

    
    //System.out.println("RUN Speaker");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
