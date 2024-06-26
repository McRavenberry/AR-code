// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Meters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.5; // 4.8
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Lower slew rates mean slower, more precise control; higher slew rates mean faster response
    public static final double kDirectionSlewRate = 1.2; // radians per second / 1.2
    public static final double kMagnitudeSlewRate = 1.5; // percent per second (1 = 100%) / 1.8
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%) / 2.0

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;
    
    public static final boolean kGyroReversed = false;
  }

  //Class for Intake Mechanism 
  public static final class IntakeConstants {
    public static final int intakeArmMotorID = 4; //TalonFX on Falcon 500
    public static final int intakeWheelMotorID = 3; //Talon SRX on 775pro
   }
   //Class for Amper mechanism 
  public static final class AmperConstants {
    public static final int ampArmMotorID = 2; 
    public static final int ampWheelMotorId = 5; 
    
   }
   //Class for Shooter 
  public static final class ShooterConstants {
    public static final int leftShooterMotorId = 6; 
    public static final int rightShooterMotorId = 7; 
    public static final double kSpeakerPower = 0.80; 
    public static final double kAmpPower = 0.20;
    
   }
   // class for vision
   public static class VisionConstants {

    /**
     * Array of PhotonVision camera names. The values here match ROBOT_TO_CAMERA_TRANSFORMS for the camera's location.
    */
    public static final String[] APRILTAG_CAMERA_NAMES = {"VisionCamera"};

    /**
     * Physical location of the apriltag cameras on the robot, relative to the center of the robot.
     * The values here math APRILTAG_CAMERA_NAMES for the camera's name.
     */
    public static final Transform3d[] ROBOT_TO_CAMERA_TRANSFORMS = {
        new Transform3d(
            new Translation3d(inchesToMeters(14), inchesToMeters(-14), inchesToMeters(20.25)),
            new Rotation3d(0, degreesToRadians(-43), degreesToRadians(0))),
      };

    public static final Measure<Distance> FIELD_LENGTH = Meters.of(16.54175);
    public static final Measure<Distance> FIELD_WIDTH = Meters.of(8.0137);

    /**
     * Minimum target ambiguity. Targets with higher ambiguity will be discarded. Not appliable when multiple tags are
     * in view in a single camera.
     */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

  }
   //Class for climber 
  public static final class ClimberConstants {
    public static final int climberMotorId = 8; 
    //public static final double climbSpeed  = 0.75;
    public static final double softLimitUpper = 0.37;
    public static final double softLimitLower = 0.0;
    public static final double sprocketDiameter = 0.034;
    public static final double climberReduction = 0.05;
    public static final double sprocketCircumference = sprocketDiameter * Math.PI;
    public static final double climbPositionFactor = (sprocketCircumference * Math.PI)
        /climberReduction; // meters
    public static final double climbVelocityFacotor = climberReduction*2.0*Math.PI/60;
    public static final double climbFreeSpeed = 5676.0*climbVelocityFacotor;
    public static final double kHomePosition = 0.0; 
    public static final double kClimbPosition = 1.5;     
   }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; // 0.0762
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final int kOperatorControlPort = 1; 
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


 }