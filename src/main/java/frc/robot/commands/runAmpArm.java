// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class runAmpArm extends Command {
  /** Creates a new ArmScore. */
  private AmperSubsystem s_Amper;
  private ShooterSubsystem s_Shooter;
  private Boolean protect = false;

  public runAmpArm(AmperSubsystem s_Amper, ShooterSubsystem s_Shooter, Boolean protect) {
    this.s_Amper = s_Amper;
    this.s_Shooter = s_Shooter;
    this.protect = protect;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Amper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(protect){
      s_Amper.setArm(0);
    }
    else{
      s_Amper.setArm(6500);
      //s_Amper.setSpeed(0.9);
      //s_Shooter.setSpeedAmp();
    }

    
    System.out.println("RUN AMP");

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
