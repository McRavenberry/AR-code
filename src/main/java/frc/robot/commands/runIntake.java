// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class runIntake extends Command {
  private IntakeSubsystem s_Intake;
  private double pos;
  private Timer timer;
  private boolean autoDone;

  /** Creates a new runIntake. */
  public runIntake(IntakeSubsystem s_Intake, double pos) {
    this.s_Intake = s_Intake;
    this.pos = pos;
    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Starts timer for the autonomous intake
    timer.reset();
    timer.start();
    autoDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.setMotor(0.9);
    s_Intake.setArm(84);

    // Triggers the intake sequence to end after 'time' seconds
    double time = 1.0;
    if(DriverStation.isAutonomous() == true && timer.get() > time){
      autoDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Resets the intake arm position and turns the motor off
     s_Intake.setArm(0);
     s_Intake.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // turns off intake when autonomous is complete
    if(DriverStation.isAutonomous()){
      return autoDone;
    }
    else{
    return false;
    }
  }
}
