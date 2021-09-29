/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Drivetrain;

public class AutomaticDrive extends CommandBase {

  Drivetrain drivetrain;

  public Timer autoTimer = new Timer();
  public double time = 5.0;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  double tx = limelightTable.getEntry("tx").getDouble(0.0);
  double ty = limelightTable.getEntry("ty").getDouble(0.0);
  double ta = limelightTable.getEntry("ta").getDouble(0.0);
  double tv = limelightTable.getEntry("tv").getDouble(0.0);

  /**
   * Creates a new DriveWithJoystick.
   */
  public AutomaticDrive(Drivetrain p_drivetrain, double p_time) {
    // Use addRequirements() here to declare subsystem dependencies.
    time = p_time
    drivetrain = p_drivetrain;
    addRequirements(drivetrain);
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
   // System.out.println("Execute Works");
    double controlConstant = -0.1;
    //double heading_error = tx;
    double autoSteerAdjustment = controlConstant * tx;
    double autoDistanceAdjustment = controlConstant * ty;
    
    leftAimInput = autoSteerAdjustment;
    rightAimInput = -1 * autoSteerAdjustment;

    leftRangeInput = autoDistanceAdjustment;
    rightRangeInput = autoDistanceAdjustment;
    
    //drivetrain.drivetrain.arcadeDrive(0, horizontalSteeringAdjustment);

    /*
    boolean cool = Robot.controllerDrive.getXButton();
    Drivetrain.driveCooler.set(cool);
    */

    if(tv < 1) {
      drivetrain.drivetrain.arcadeDrive(0, 0.5);
    } else {
      if (ty == 0.0) {
        drivetrain.drivetrain.tankDrive(leftAimInput, rightAimInput);
      } else {
        drivetrain.drivetrain.tankDrive(leftRangeInput, rightRangeInput);
      } 
    }

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
    return autoTimer.get() >= time;
  }
}
