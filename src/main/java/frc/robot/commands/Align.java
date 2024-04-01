package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class Align extends Command {
  private DriveSubsystem s_DriveSubsystem;
  private CommandXboxController controller;
  //private double pValue = 8 / 42; 
  PIDController turnController = new PIDController(0.0025, 0, 0); //FIXME This will need to your robot
  private double rotationVal;
  private boolean blueAlliance;

  /** Creates a new Align. */
  public Align(DriveSubsystem s_DriveSubsystem, CommandXboxController controller) {
    this.s_DriveSubsystem = s_DriveSubsystem;
    addRequirements(s_DriveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Gets current Alliance
    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Blue){
        blueAlliance = true;
      }
      else{
        blueAlliance = false;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(blueAlliance){
        rotationVal = turnController.calculate(s_DriveSubsystem.getHeading(), 90);//pValue * (90 - (s_DriveSubsystem.getHeading() + 3600000) % 360); //Change 90 or 270 to snap to different angle
      }
      else{
        rotationVal =  turnController.calculate(s_DriveSubsystem.getHeading(), 270);
      }
    s_DriveSubsystem.drive(
                -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband),
                rotationVal,
                true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}