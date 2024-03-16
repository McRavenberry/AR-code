// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Searcy Robot 2024
// Truffle Shuffle

package frc.robot;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AmperSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import javax.swing.text.html.HTMLDocument.RunElement;

import com.fasterxml.jackson.databind.jsontype.BasicPolymorphicTypeValidator.NameMatcher;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.commands.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final AmperSubsystem m_AmperSubsystem = new AmperSubsystem();
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  // Sets up the driver and operator controllers
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_OperatorController = new CommandXboxController(OIConstants.kOperatorControlPort);

  //operator controller 
  Joystick m_OperatorJoystick = new Joystick(OIConstants.kOperatorControlPort);

  Command runIntake = new runIntake(m_IntakeSubsystem, 10);
  Command intakeOut = new intakeOut(m_IntakeSubsystem, 60);
  Command runAmpArm = new runAmpArm(m_AmperSubsystem, m_shooter, m_IntakeSubsystem, false);
  Command runAmpArmProtect = new runAmpArm(m_AmperSubsystem, m_shooter, m_IntakeSubsystem, true);
  Command runAmpWheel = new runAmpWheel(m_AmperSubsystem, m_shooter, m_IntakeSubsystem);
  Command runShooter = new runShooter(m_IntakeSubsystem, m_shooter, true);
  Command AutoCommand = new AutoShoot(m_IntakeSubsystem, m_shooter, null, 0);

  // Makes sendablechooser for autonomous routines
  SendableChooser<Command> auto;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Register named commands for PathPlanner
    // NamedCommands.registerCommand("speaker", Commands.print("speaker"));
    // NamedCommands.registerCommand("pickup", Commands.print("pickup note"));
    // NamedCommands.registerCommand("amp", Commands.print("amp"));
    // NamedCommands.registerCommand("load", Commands.print("load"));
    NamedCommands.registerCommand("shoot", AutoCommand);
    NamedCommands.registerCommand("intake", runIntake);
    NamedCommands.registerCommand("shootEnd", runShooter);
    NamedCommands.registerCommand("deployIntake", new deployIntake(m_IntakeSubsystem));

    // Sets up AutoBuilder from PathPlanner and displays auto choices onto SmartDashboard
    auto = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Choose", auto);

    // Configure the button bindings
    configureButtonBindings();
    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
        
    
        
  }     

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController.getHID(), Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    //Shoot into AMP with left trigger on Joystick 
    m_OperatorController.button(6).onTrue(runAmpArm);
    m_OperatorController.button(5).onTrue(runAmpArmProtect);
    m_OperatorController.button(4).onTrue(runAmpWheel);

    //Pick up note off ground (Intake)
    m_OperatorController.button(7).whileTrue(runIntake);

  
    //Spit out note (Intake)
    m_OperatorController.button(2).whileTrue(intakeOut);

    //Shoot into SPEAKER with left trigger 
    m_OperatorController.button(8).whileTrue(runShooter);

    //Upper and Lower positions for climber
    m_OperatorController.povUp().onTrue(new InstantCommand(() -> m_ClimberSubsystem.setMotors(110)));
    m_OperatorController.povDown().onTrue(new InstantCommand(() -> m_ClimberSubsystem.setMotors(1)));

    //Button 5 on driver controller toggles between field centric and robot centric driving
    m_driverController.button(5).onTrue(new InstantCommand(() -> m_robotDrive.fieldRelative()));
    m_driverController.button(4).onTrue(new InstantCommand(()-> m_robotDrive.zeroHeading()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return auto.getSelected();
  }
}