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
import edu.wpi.first.wpilibj.Timer;

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
    time = p_time;
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
   
    double aimControlConstant = -0.1;
    double distanceControlConstant = -0.1;
    double min_aim_command = 0.05;

    double leftInput;
    double rightInput;
    
    /*
    boolean cool = Robot.controllerDrive.getXButton();
    Drivetrain.driveCooler.set(cool);
    */

/*
    if(tv < 1) {
      drivetrain.drivetrain.arcadeDrive(0, 0.5);
    } else {
      if (ty == 0.0) {
        drivetrain.drivetrain.tankDrive(leftAimInput, rightAimInput);
      } else {
        drivetrain.drivetrain.tankDrive(leftRangeInput, rightRangeInput);
      } 
    }

    if(Robot.shooting) {
        if(Robot.flywheelVel < Flywheel.velocityTarget) {
          flywheel.flywheel1.set(1.0);
        } else {
          flywheel.flywheel1.set(0.925);//og 0.95
        }
        turret.hoodPID.setReference(9.5, ControlType.kPosition); //1.8 originally, changed to 2.0, changed to 2.2, changed 2.3, changed 2.5, changed 2.8, changed 3.0
        Robot.shooting = true;
    } else {
        //flywheel.disable();
        flywheel.flywheel1.set(0.0);
    }
*/

    if (tv == 1)
    {
        double heading_error = -1 * tx;
        double distance_error = -1 * ty;
        double steering_adjust;

        if (tx > 1.0)
        {
            steering_adjust = aimControlConstant * heading_error - min_aim_command;
        }
        else if (tx < 1.0)
        {
            steering_adjust = aimControlConstant * heading_error + min_aim_command;
            turret.hoodPID.setReference(9.5, ControlType.kPosition); //1.8 originally, changed to 2.0, changed to 2.2, changed 2.3, changed 2.5, changed 2.8, changed 3.0
            Robot.shooting = true;
        }

        double distance_adjust = distanceControlConstant * distance_error;

        leftInput += steering_adjust + distance_adjust;
        rightInput -= steering_adjust + distance_adjust;

        drivetrain.drivetrain.tankDrive(leftInput, rightInput);
    } else {
        drivetrain.drivetrain.arcadeDrive(0, 0.5);
    }

    if(Robot.shooting) {
        if(Robot.flywheelVel < Flywheel.velocityTarget) {
          flywheel.flywheel1.set(1.0);
        } else {
          flywheel.flywheel1.set(0.925);//og 0.95
        }
        turret.hoodPID.setReference(9.5, ControlType.kPosition); //1.8 originally, changed to 2.0, changed to 2.2, changed 2.3, changed 2.5, changed 2.8, changed 3.0
        Robot.shooting = true;
    } else {
        flywheel.flywheel1.set(0.0);
    }

    
    Drivetrain.falconTempDashboard[0].setDouble(drivetrain.frontLeft.getTemperature());
    Drivetrain.falconTempDashboard[1].setDouble(drivetrain.rearLeft.getTemperature());
    Drivetrain.falconTempDashboard[2].setDouble(drivetrain.frontRight.getTemperature());
    Drivetrain.falconTempDashboard[3].setDouble(drivetrain.rearRight.getTemperature());


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
