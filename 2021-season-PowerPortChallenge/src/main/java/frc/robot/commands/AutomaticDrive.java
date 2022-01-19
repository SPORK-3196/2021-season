/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Turret;

public class AutomaticDrive extends CommandBase {

  private final Drivetrain drivetrain;
  private final Turret turret;
  private final Flywheel flywheel;


  public Timer autoTimer = new Timer();
  public double time = 5.0;

 
  NetworkTable robotTable = NetworkTableInstance.getDefault().getTable("Default");


  
  boolean shootDuringAuto = robotTable.getEntry("Shoot during auto?").getBoolean(false);


  /**
   * Creates a new DriveWithJoystick.
   */
  public AutomaticDrive(Drivetrain p_drivetrain, double p_time, Turret newTurret, Flywheel newFlywheel) {
    // Use addRequirements() here to declare subsystem dependencies.
    time = p_time;
    drivetrain = p_drivetrain;
    turret = newTurret;
    flywheel = newFlywheel;
    addRequirements(drivetrain);
    addRequirements(turret);
    addRequirements(flywheel);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drivetrain.setDeadband(0.1);

    drivetrain.frontRight.setNeutralMode(NeutralMode.Coast);
    drivetrain.rearRight.setNeutralMode(NeutralMode.Coast);
    drivetrain.frontLeft.setNeutralMode(NeutralMode.Coast);
    drivetrain.rearLeft.setNeutralMode(NeutralMode.Coast);
    autoTimer.reset();
    autoTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isTargetVisible = false;
    double aimControlConstant = -0.04;
    double distanceControlConstant = -0.1;
    double min_aim_command = 0;

    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    double tx = limelightTable.getEntry("tx").getValue().getDouble();
    double ty = limelightTable.getEntry("ty").getValue().getDouble();
    double ta = limelightTable.getEntry("ta").getValue().getDouble();
    double tv = limelightTable.getEntry("tv").getValue().getDouble();

    double heading_error = -1 * tx;
    double distance_error = -1 * ty;
    double steering_adjust = 0.0;

    double leftInput = 0.0;
    double rightInput = 0.0;
    double distance_adjust = 0.0;

    if (tv == 1) {
      isTargetVisible = true;
    }
    else if (tv == 0) {
      isTargetVisible = false;
    }

    if (isTargetVisible) {
      if (tx > 1.0) {
        steering_adjust = (aimControlConstant * heading_error) - min_aim_command;
      }
      else if (tx < 1.0) {
        steering_adjust = (aimControlConstant * heading_error) + min_aim_command;
      }
    }
    else if (!isTargetVisible) {
      drivetrain.drivetrain.arcadeDrive(0, 0.1);
    }

    distance_adjust = distanceControlConstant * distance_error;
    
    
    leftInput += steering_adjust + distance_adjust;
    rightInput -= steering_adjust + distance_adjust;

    drivetrain.drivetrain.tankDrive(leftInput, rightInput);
    
   
    Drivetrain.falconTempDashboard[0].setDouble(drivetrain.frontLeft.getTemperature());
    Drivetrain.falconTempDashboard[1].setDouble(drivetrain.rearLeft.getTemperature());
    Drivetrain.falconTempDashboard[2].setDouble(drivetrain.frontRight.getTemperature());
    Drivetrain.falconTempDashboard[3].setDouble(drivetrain.rearRight.getTemperature());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return autoTimer.get() >= time;
    return false;
  }
}
