package frc.robot.commands;

import java.util.function.BiPredicate;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class runShooter extends Command {
  /** Creates a new Shooter. */
    private ShooterSubsystem s_Shooter;
    private IntakeSubsystem s_Intake;
    private Timer timer;
    private boolean autoDone;

  public runShooter(IntakeSubsystem s_Intake, ShooterSubsystem s_Shooter, Boolean protect2) {
    this.s_Intake = s_Intake;
    this.s_Shooter = s_Shooter;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // For autonomous to start timer to shut off shooter
    timer.reset();
    timer.start();
    autoDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //s_Shooter.speakerShoot();
    s_Shooter.runShooter(5200);

    if(s_Shooter.getAtSpeed(5200)){
      s_Intake.setMotor(-1);
    }

    // Triggers the shooter motors to stop after 'time' seconds
    double time = 2.0;
    if(DriverStation.isAutonomous() == true && timer.get() > time){
      autoDone = true;
    }

    System.out.println("run Shooter");
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Turns shooter motors off
    s_Shooter.stopShooter();
    s_Intake.setMotor(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Turns off shooter motor when autonomous shooting is complete
    if(DriverStation.isAutonomous()){
      return autoDone;
    }
    else{
      return false;
    }
  }
}
