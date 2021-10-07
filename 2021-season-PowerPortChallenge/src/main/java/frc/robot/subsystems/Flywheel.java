/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Robot;

public class Flywheel extends PIDSubsystem {
  
  public WPI_TalonSRX flywheel1 = new WPI_TalonSRX(9);
  public WPI_TalonSRX flywheel2 = new WPI_TalonSRX(10);

  public static int velocityTarget = 360; // og 340
  Index index;

  /**
   * Creates a new Flywheel.
   */
  public Flywheel() {
    super(
        // The PIDController used by the subsystem
        new PIDController(2.0, 0.0, 0.5));
    
    flywheel2.follow(flywheel1);
    flywheel1.setInverted(true);
    flywheel2.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    
    
    //run fkywheel by itself - uses right bumber of secondary controller
    boolean maxPower = Robot.controllerSecondary.getBumper(Hand.kRight);
    if(maxPower) {
      flywheel1.set(1.0);
      if (flywheel1.getSelectedSensorVelocity() == 300) {
        index.runMotorsShooting();
      }
      //ifStatementAddedFeb24TryingToRunFlywheelAndIndexWhenPressingBumberOnController
    } else {
      flywheel1.set(0); //addedFeb24,2021

    // Use the output here
    if(output <= 0.0) {
      if(Robot.shooting) {
        flywheel1.set(1.0);//og 0.95

      } else {
        flywheel1.set(0.0);
      }
    }
    else {
      flywheel1.set(output); 
    }
    }
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return flywheel1.getSelectedSensorVelocity();
  }
}
