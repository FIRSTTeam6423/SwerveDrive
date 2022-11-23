// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveUtil extends SubsystemBase {
  //P denotes Pivoting, D driving

  private final Translation2d m_frontLeftLoc = new Translation2d(Constants.TOPLEFT_X, Constants.TOPLEFT_Y);
  private final Translation2d m_frontRightLoc = new Translation2d(Constants.TOPRIGHT_X, Constants.TOPRIGHT_Y);
  private final Translation2d m_backLeftLoc = new Translation2d(Constants.BOTTOMLEFT_X, Constants.TOPLEFT_Y);
  private final Translation2d m_backRightLoc = new Translation2d(Constants.BOTTOMRIGHT_X, Constants.BOTTOMRIGHT_Y);

  private final SwerveModule m_frontLeft =  new SwerveModule(Constants.TOPLEFT_DRIVE, Constants.TOPLEFT_PIVOT);
  private final SwerveModule m_frontRight =  new SwerveModule(Constants.TOPRIGHT_DRIVE, Constants.TOPRIGHT_PIVOT);
  private final SwerveModule m_backLeft =  new SwerveModule(Constants.BOTTOMLEFT_DRIVE, Constants.BOTTOMLEFT_PIVOT);
  private final SwerveModule m_backRight =  new SwerveModule(Constants.BOTTOMRIGHT_DRIVE, Constants.BOTTOMRIGHT_PIVOT);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLoc, m_frontRightLoc, m_backLeftLoc, m_backRightLoc);


  private ChassisSpeeds speeds;

  public double setpoint;

  public DriveUtil() {
  setpoint = 0;

  }

  public void driveRobot(){
    //Scaling needed for inputs to speeds - 1 m/s is not max we want
    speeds = new ChassisSpeeds(RobotContainer.getDriverLeftXboxX() * Constants.MAX_LINEAR_SPEED, 
    RobotContainer.getDriverLeftXboxY() * Constants.MAX_LINEAR_SPEED,
    RobotContainer.getDriverRightXboxX() * Constants.MAX_ANGULAR_SPEED);
    
    var SwerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);
    m_frontLeft.setDesiredState(SwerveModuleStates[0]);
    m_frontRight.setDesiredState(SwerveModuleStates[1]);
    m_backLeft.setDesiredState(SwerveModuleStates[2]);
    m_backRight.setDesiredState(SwerveModuleStates[3]);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
