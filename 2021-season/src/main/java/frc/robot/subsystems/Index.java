/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;


public class Index extends SubsystemBase {
  public CANSparkMax intake = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax firstStage = new CANSparkMax(6, MotorType.kBrushless);
  public CANSparkMax secondStage = new CANSparkMax(7, MotorType.kBrushless);

 // public CANSparkMax intake = null;
 // public CANSparkMax firstStage = null;
 // public CANSparkMax secondStage = null;

  public static Solenoid intakeSolenoid1 = new Solenoid(50, 0);
  public static Solenoid intakeSolenoid2 = new Solenoid(50, 7);

  public boolean intakeOut = false;

  public static DigitalInput[] sensor = new DigitalInput[10];
  int counter = 0;
  public boolean waiting = false;
  public boolean lastSensor0Value = false;
  boolean lastSensor1Value = false;
  public boolean lastSensor5Value = false;
  public boolean loaded = false;

  public static NetworkTableEntry counter_D = Shuffleboard.getTab("Default").add("Index", 0).getEntry();
  public static NetworkTableEntry waiting_D = Shuffleboard.getTab("Default").add("Waiting", false).getEntry();
  public static NetworkTableEntry loaded_D = Shuffleboard.getTab("Default").add("Loaded", false).getEntry();


  public boolean getSensorValue(int sensorNumber) {
    return !sensor[sensorNumber].get();
  }

  public void reset() {
    counter = 0;
    waiting = false;
    lastSensor0Value = false;
    lastSensor1Value = false;
    lastSensor5Value = false;
  }

  public void runMotors() {
    firstStage.set(0.32);
    secondStage.set(0.5);
  }

  public void runMotorsShooting() {
    firstStage.set(0.7);
    secondStage.set(1.0);
  }

  public void stopMotors() {
    firstStage.set(0.0);
    secondStage.set(0.0);
  }

  public void runIntake() {
    boolean reverseIntake = Robot.controllerSecondary.getBumper(Hand.kRight);
    if(reverseIntake) {
      intake.set(-0.5); // Reverse intake
    } else if(waiting) {
      intake.set(0.25); // Run intake slower to avoid indexing problems
    } else if(intakeOut) {
      intake.set(0.5); // Run intake normally
    } else {
      intake.set(0.0); // Stop intake because it's retracted
    }
  }

  public void index() {
    counter++;
    waiting = true;
  }

  public void run() {
    if(waiting) {
      runMotors();
      if((counter < 5) && (!lastSensor1Value) && getSensorValue(1)) {
        waiting = false;
        stopMotors();
      } else if(!getSensorValue(5) && lastSensor5Value) {
        waiting = false;
        loaded = true;
        stopMotors();
        intakeOut = false;
      }
    } else {
      stopMotors();
    }
    runIntake();
    Index.intakeSolenoid1.set(intakeOut);
    Index.intakeSolenoid2.set(intakeOut);

    if(Robot.shooting) {
      if(Robot.deltaFlywheelVel < -3) {
        loaded = false;
      }

      if((Robot.flywheelVel > (Flywheel.velocityTarget-1) /*&& Math.abs(Robot.turretError) < 10*/) || !loaded) {
        runMotorsShooting();
        reset();
      } else {
        stopMotors();
      }
    }

    if(!loaded) {
      if(getSensorValue(5)) {
        loaded = true;
        stopMotors();
      }
    }

    double expel = Robot.controllerSecondary.getTriggerAxis(Hand.kRight);
    if(expel > 0.1) {
      firstStage.set(expel * -0.5);
      secondStage.set(expel * -0.5);
      reset();
      loaded = false;
    }

    lastSensor0Value = getSensorValue(0);
    lastSensor1Value = getSensorValue(1);
    lastSensor5Value = getSensorValue(5);
    counter_D.setNumber(counter);
    waiting_D.setBoolean(waiting);
    loaded_D.setBoolean(loaded);
  }
  
  /**
   * Creates a new Index.
   */
  public Index() {
    //intake = new CANSparkMax(5, MotorType.kBrushless);
    //firstStage = new CANSparkMax(6, MotorType.kBrushless);
    //secondStage = new CANSparkMax(7, MotorType.kBrushless);
    

  
    for(int i = 0; i < 10; i++){
      if(sensor[i] == null) { 
        sensor[i] = new DigitalInput(i);
      }
    }
    
    intakeOut = false;
    intake.set(0.0);
    Index.intakeSolenoid1.set(false);
    Index.intakeSolenoid2.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
