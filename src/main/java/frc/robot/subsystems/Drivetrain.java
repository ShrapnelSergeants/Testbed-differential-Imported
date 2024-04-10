// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PhysicalConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(DrivetrainConstants.kFrontLeftMotorID);
  private final WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(DrivetrainConstants.kFrontRightMotorID);

  private final VictorSPX backLeftDrive = new VictorSPX(DrivetrainConstants.kBackLeftMotorID);
  private final VictorSPX backRightDrive = new VictorSPX(DrivetrainConstants.kBackRightMotorID);

  private final DifferentialDrive differentialDrive = new DifferentialDrive(frontLeftDrive, frontRightDrive);

  private final Encoder frontLeftEncoder = new Encoder(DrivetrainConstants.kFrontLeftEncoderA, DrivetrainConstants.kFrontLeftEncoderB, false, EncodingType.k4X);
  //private final Encoder backLeftEncoder = new Encoder(DrivetrainConstants.kBackLeftEncoderA, DrivetrainConstants.kBackLeftEncoderB, false, EncodingType.k4X);
  private final Encoder frontRightEncoder = new Encoder(DrivetrainConstants.kFrontRightEncoderA, DrivetrainConstants.kFrontRightEncoderB, false, EncodingType.k4X);
  //private final Encoder backRightEncoder = new Encoder(DrivetrainConstants.kBackRightEncoderA, DrivetrainConstants.kBackRightEncoderB, false, EncodingType.k4X);   


  private final AHRS gyro;

  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getAngle(),getLeftDistanceMeters(),getRightDistanceMeters());

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.PhysicalConstants.kTrackWidth);

  private DifferentialDriveWheelPositions wheelPositions;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.22, 1.98, 0.2); //kS, kV, kA - get from other code
  
  private final PIDController leftPIDController = new PIDController(8.5,0,0); //update values
  private final PIDController rightPIDController = new PIDController(8.5,0,0); //update values
  private Field2d field = new Field2d();

  public Drivetrain() {
    //left side
    frontLeftDrive.configFactoryDefault();
    backLeftDrive.configFactoryDefault();

    frontLeftDrive.setNeutralMode(NeutralMode.Brake);
    backLeftDrive.setNeutralMode(NeutralMode.Brake);

    frontLeftDrive.setInverted(DrivetrainConstants.kFrontLeftMotorInverted);
    backLeftDrive.setInverted(DrivetrainConstants.kBackLeftMotorInverted);

    backLeftDrive.follow(frontLeftDrive);
 
    //right side
    frontRightDrive.configFactoryDefault();
    backRightDrive.configFactoryDefault();

    frontRightDrive.setNeutralMode(NeutralMode.Brake);
    backRightDrive.setNeutralMode(NeutralMode.Brake);

    frontRightDrive.setInverted(DrivetrainConstants.kFrontRightMotorInverted);
    backRightDrive.setInverted(DrivetrainConstants.kBackRightMotorInverted);

    backRightDrive.follow(frontRightDrive);


    wheelPositions = new DifferentialDriveWheelPositions(
                        getLeftDistanceMeters(),
                        getRightDistanceMeters()
    );


    //gyro
    gyro = new AHRS();
    
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          gyro.reset();
      } catch (Exception e) {
      }
  }).start();
  

    frontLeftEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);
    frontRightEncoder.setDistancePerPulse(PhysicalConstants.kDistancePerPulse);
/*
    AutoBuilder.configureRamsete(
      this::getPose,
      this::resetPose,
      this::getCurrentSpeeds,
      this::drive,
      new ReplanningConfig(),
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }
    
      
    )
*/

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getAngle(), getLeftDistanceMeters(), getRightDistanceMeters());
  }



    /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    frontLeftDrive.setVoltage(leftVolts);
    frontRightDrive.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftEncoder.reset();
    frontRightEncoder.reset();
  }

  public Rotation2d getAngle(){
    return gyro.getRotation2d();
  }

  public double getLeftDistanceMeters(){
    return frontLeftEncoder.getDistance();
  }

  public double getRightDistanceMeters(){
    return frontRightEncoder.getDistance();
  }

  /**
   * Gets the average distance of the TWO encoders.
   *
   * @return the average of the TWO encoder readings
   */
  public double getAverageEncoderDistance() {
    return (frontLeftEncoder.getDistance() + frontRightEncoder.getDistance()) / 2.0;
  }

    /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public PIDConstants getLeftPIDConstants(){
    return new PIDConstants(8.5,0,0); //update

  }

  public PIDConstants getRightPIDConstants(){
    return new PIDConstants(8.5,0,0); //update
  }

  public SimpleMotorFeedforward getSimpleMotorFeedforward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }
/*
  public Pose2d resetPose(){
    return odometry.resetPosition(getAngle(), wheelPositions, getPose());
  }
*/
  public void resetOdometry(){
    odometry.resetPosition(getAngle(), 0, 0, getPose());

  }


  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(frontLeftEncoder.getRate(), frontRightEncoder.getRate());
  }

  /*
  public PPRamseteCommand createRamseteCommand(PathPlannerTrajectory trajectory){
    return new PPRamseteCommand(
      trajectory,
      this::getPose(),
      new RamseteController(2, 0.7), 
      feedforward,
      kinematics,
      this::getWheelSpeeds,
      leftPIDController,
      rightPIDController,
      this::tankDriveVolts,
      this
    );
  }
*/

  // look into this
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    // Publish encoder distances to telemetry.
    builder.addDoubleProperty("leftDistance", frontLeftEncoder::getDistance, null);
    builder.addDoubleProperty("rightDistance", frontRightEncoder::getDistance, null);
  }
}
