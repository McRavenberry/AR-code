// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class runIntake extends Command {
  private IntakeSubsystem s_Intake;
  private double pos;

  /** Creates a new runIntake. */
  public runIntake(IntakeSubsystem s_Intake, double pos) {
    this.s_Intake = s_Intake;
    this.pos = pos;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.setMotor(0.9);
    s_Intake.setArm(84);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     s_Intake.setArm(0);
     s_Intake.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
