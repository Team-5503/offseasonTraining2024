// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.LTVUnicycleController;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import com.kauailabs.navx.frc.AHRS;

public class driveTrain extends SubsystemBase {
  private CANSparkMax frontR, backR, frontL, backL;
  private AHRS gyro;
  /** Creates a new driveTrain. */
  private DifferentialDriveOdometry m_odometry;
  RelativeEncoder leftEncoder = frontL.getEncoder();
  RelativeEncoder rightEncoder = frontR.getEncoder();

  public driveTrain() {
    frontR = new CANSparkMax(0, MotorType.kBrushless);
    frontL = new CANSparkMax(1, MotorType.kBrushless);
    backR = new CANSparkMax(2, MotorType.kBrushless);
    backL = new CANSparkMax(3, MotorType.kBrushless);
    backL.follow(frontL);
    backR.follow(frontR);
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }
  public double getRightEncoderPosition() {
    return -rightEncoder.getPosition();
  }
  public double getLeftEncoderPosition() {
    return leftEncoder.getPosition();
  }
  public void resetOdometry(Pose2d pose) {
    AHRS gyro = new AHRS();
    resetEncoders();
    m_odometry.resetPosition(gyro.getRotation2d(), getLeftEncoderPosition(),
        getRightEncoderPosition(), pose);
  }
  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return leftEncoder.getVelocity();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void resetPose(Pose2d pose) {
    resetOdometry(pose);
  }
public DifferentialDriveWheelSpeeds getRobotRelativeSpeeds() {
  return getWheelSpeeds();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(double forward ,double turn){
    double leftSpd = forward + turn;
    double rightSpd = forward - turn;

    frontL.set(leftSpd);
    frontR.set(rightSpd);

  }
}

