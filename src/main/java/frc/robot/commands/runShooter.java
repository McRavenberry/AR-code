package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class runShooter extends Command {
  /** Creates a new Shooter. */
    private ShooterSubsystem s_Shooter;
    private IntakeSubsystem s_Intake;

  public runShooter(IntakeSubsystem s_Intake, ShooterSubsystem s_Shooter, Boolean protect2) {
    this.s_Intake = s_Intake;
    this.s_Shooter = s_Shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //s_Shooter.speakerShoot();
    s_Shooter.runShooter(4000);

    if(s_Shooter.getAtSpeed(4000)){
      s_Intake.setMotor(-1);
    }

    System.out.println("run Shooter");
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.stopShooter();
    s_Intake.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
