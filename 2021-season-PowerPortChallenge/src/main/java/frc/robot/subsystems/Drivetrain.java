/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.net.Socket;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  public WPI_TalonFX frontLeft = new WPI_TalonFX(1);
  public WPI_TalonFX rearLeft = new WPI_TalonFX(2);
  public WPI_TalonFX frontRight = new WPI_TalonFX(3);
  public WPI_TalonFX rearRight = new WPI_TalonFX(4);

  SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, rearLeft);
  SpeedControllerGroup right = new SpeedControllerGroup(frontRight, rearRight);

  public DifferentialDrive drivetrain = new DifferentialDrive(left, right);

  public Orchestra orchestra;
  public String song;

  //public static Solenoid driveCooler = new Solenoid(50, 4);

  public static NetworkTableEntry[] falconTempDashboard = new NetworkTableEntry[4];
  public static NetworkTableEntry rightEncoderDashboard = Shuffleboard.getTab("Default").add("Right Falcon Encoder", 0.0).getEntry();
  public static NetworkTableEntry leftEncoderDashboard = Shuffleboard.getTab("Default").add("Left Falcon Encoder", 0.0).getEntry();
  public static NetworkTableEntry leftRightDifferenceDashboard = Shuffleboard.getTab("Default").add("LeftRight Difference", 0.0).getEntry();

  public static NetworkTableEntry musicTrackTime = Shuffleboard.getTab("Default").add("Music Track Time", 0.0).getEntry();

  public int rightEncoderOffset = 0;
  public int leftEncoderOffset = 0;
  public int leftRightDifference = 0;

  public int getRightEncoderPosition() {
   // return (int) (-frontRight.getSelectedSensorPosition() - rightEncoderOffset);
   return 0;
  }

  public int getLeftEncoderPosition() {
   // return (int) (rearLeft.getSelectedSensorPosition() - leftEncoderOffset);
   return 0;
  }

  public void resetEncoders() {
    rightEncoderOffset += getRightEncoderPosition();
    leftEncoderOffset += getLeftEncoderPosition();
    leftRightDifference = getLeftEncoderPosition() - getRightEncoderPosition();
  }

  public void playMusic() {
    orchestra.play();
  }

  public void pauseMusic() {
    orchestra.pause();
  }

  public void stopMusic() {
    orchestra.stop();
  }

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    drivetrain.setRightSideInverted(true);

    falconTempDashboard[0] = Shuffleboard.getTab("Default").add("FL Falcon Temp", 0.0).getEntry();
    falconTempDashboard[1] = Shuffleboard.getTab("Default").add("RL Falcon Temp", 0.0).getEntry();
    falconTempDashboard[2] = Shuffleboard.getTab("Default").add("FR Falcon Temp", 0.0).getEntry();
    falconTempDashboard[3] = Shuffleboard.getTab("Default").add("RR Falcon Temp", 0.0).getEntry();

    resetEncoders();

    ArrayList<TalonFX> fx_s = new ArrayList<TalonFX>();
    fx_s.add(frontLeft);
    fx_s.add(rearLeft);
    fx_s.add(frontRight);
    fx_s.add(rearRight);
    
    song = "Crab.chrp";
    orchestra = new Orchestra(fx_s, song);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightEncoderDashboard.setDouble(getRightEncoderPosition());
    leftEncoderDashboard.setDouble(getLeftEncoderPosition());
    musicTrackTime.setDouble(orchestra.getCurrentTime());

    leftRightDifference = getLeftEncoderPosition() - getRightEncoderPosition();
    leftRightDifferenceDashboard.setDouble(leftRightDifference);

    //System.out.println("Periodic Works");

  }
}
