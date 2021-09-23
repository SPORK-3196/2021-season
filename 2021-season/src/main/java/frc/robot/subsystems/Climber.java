package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
    public CANSparkMax leftClimberMotor = new CANSparkMax(5, MotorType.kBrushless);
    public CANSparkMax rightClimberMotor = new CANSparkMax(6, MotorType.kBrushless);

    public static Solenoid intakeSolenoid1 = new Solenoid(50, 4);

    public void runMotorsForward(double power) {
        leftClimberMotor.set(power);
        rightClimberMotor.set(power);
    }
    public void runMotorsBackward(double power) {
        leftClimberMotor.set(-1 * power);
        rightClimberMotor.set(-1* power);
    }
    public void stopMotors() {
        leftClimberMotor.set(0.0);
        rightClimberMotor.set(0.0);
    }

    public void spinClockwise() {
        boolean clockwiseSpin = Robot.controllerSecondary.getBumper(Hand.kRight);
        if(clockwiseSpin) {
            runMotorsForward(0.1); // Reverse intake
        }
      }

    public void spinCounterClockwise() {
        boolean counterclockSpin = Robot.controllerSecondary.getBumper(Hand.kLeft);
        if(counterclockSpin) {
            runMotorsBackward(0.1); // Reverse intake
        }
      }
}