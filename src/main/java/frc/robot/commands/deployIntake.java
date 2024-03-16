// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.DriverStation;

public class deployIntake extends Command {
  private IntakeSubsystem s_Intake;
  private boolean autoDone;

  /** Creates a new runIntake. */
  public deployIntake(IntakeSubsystem s_Intake) {
    this.s_Intake = s_Intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Starts timer for the autonomous intake
    autoDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.setArm(83);

    // Triggers the intake sequence to end after 'time' seconds
    if(s_Intake.getArm() > 80){
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
