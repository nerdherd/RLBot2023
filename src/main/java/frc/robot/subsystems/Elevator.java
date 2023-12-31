// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.NerdyMath;

public class Elevator extends SubsystemBase implements Reportable{

  private TalonFX elevator;
  private int targetTicks = ElevatorConstants.kElevatorStow;
  public BooleanSupplier atTargetPosition;
  public DoubleSupplier armAngleSupplier;
  // private DigitalInput hallEffect; // not detecting is true, detecting is false

  /** Creates a new Elevator. */
  public Elevator(DoubleSupplier armAngleSupplier) {
    elevator = new TalonFX(ElevatorConstants.kElevatorID);
    elevator.setNeutralMode(NeutralMode.Brake);
    elevator.setInverted(false);
    // hallEffect = new DigitalInput(ElevatorConstants.kHallEffectID);
    this.armAngleSupplier = armAngleSupplier;
    atTargetPosition = () -> NerdyMath.inRange(elevator.getSelectedSensorPosition(), targetTicks - 40000, targetTicks + 40000);
    // For tuning Elevator PID
    // SmartDashboard.putNumber("Elevator kP", ElevatorConstants.kElevatorP);
    // SmartDashboard.putNumber("Elevator kI", ElevatorConstants.kElevatorI);
    // SmartDashboard.putNumber("Elevator kD", ElevatorConstants.kElevatorD);
    // SmartDashboard.putNumber("Elevator kF", ElevatorConstants.kElevatorF);
    // SmartDashboard.putNumber("Elevator Accel", ElevatorConstants.kElevatorMotionAcceleration);
    // SmartDashboard.putNumber("Elevator Cruise Vel", ElevatorConstants.kElevatorCruiseVelocity);
    elevator.config_kP(0, ElevatorConstants.kElevatorP.get());
    elevator.config_kI(0, ElevatorConstants.kElevatorI.get());
    elevator.config_kD(0, ElevatorConstants.kElevatorD.get());
    elevator.config_kF(0, ElevatorConstants.kElevatorF.get());
    elevator.configMotionAcceleration(ElevatorConstants.kElevatorMotionAcceleration);
    elevator.configMotionCruiseVelocity(ElevatorConstants.kElevatorCruiseVelocity);
  }

  public void init() {
    elevator.config_kP(0, ElevatorConstants.kElevatorP.get());
    elevator.config_kI(0, ElevatorConstants.kElevatorI.get());
    elevator.config_kD(0, ElevatorConstants.kElevatorD.get());
    elevator.config_kF(0, ElevatorConstants.kElevatorF.get());
    elevator.configMotionAcceleration(ElevatorConstants.kElevatorMotionAcceleration);
    elevator.configMotionCruiseVelocity(ElevatorConstants.kElevatorCruiseVelocity);
  
  }

  @Override
  public void periodic() {
    moveMotionMagic();
  }

  public void moveElevatorJoystick(double currentJoystickOutput, double angle) {
    setBrakeMode();
      if (currentJoystickOutput > ElevatorConstants.kElevatorDeadband.get()) {
        // if (!hallEffect.get() || elevator.getSelectedSensorPosition() <= -239000 || elevator.getStatorCurrent() >= 45) {
        if (elevator.getSelectedSensorPosition() <= -239000 || elevator.getStatorCurrent() >= 45) {
        elevator.set(ControlMode.PercentOutput, 0);
        } else {
          elevator.set(ControlMode.PercentOutput, -0.8);
        }
        

        // elevator.setNeutralMode(NeutralMode.Coast);
        //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
      } else if (currentJoystickOutput < -ElevatorConstants.kElevatorDeadband.get()) {
        if (elevator.getSelectedSensorPosition() >= ElevatorConstants.kElevatorStow - 20000 || elevator.getStatorCurrent() >= 45) {
          elevator.set(ControlMode.PercentOutput, 0);
        } else {
          elevator.set(ControlMode.PercentOutput, 0.8);
        }
        
        // elevator.setNeutralMode(NeutralMode.Coast);
    
        //((currentJoystickOutput * ArmConstants.kJoystickMultiplier)));
      } else {
        // if (limitSwitch.get()) {
        elevator.set(ControlMode.PercentOutput, 0);
        
      }

    }

    
  public void moveMotionMagic() {
    // elevator.config_kP(0, SmartDashboard.getNumber("Elevator kP", ElevatorConstants.kElevatorP));
    // elevator.config_kI(0, SmartDashboard.getNumber("Elevator kI", ElevatorConstants.kElevatorI));
    // elevator.config_kD(0, SmartDashboard.getNumber("Elevator kD", ElevatorConstants.kElevatorD));
    // elevator.config_kF(0, SmartDashboard.getNumber("Elevator kF", ElevatorConstants.kElevatorF));
    // elevator.configMotionAcceleration(SmartDashboard.getNumber("Elevator Accel", ElevatorConstants.kElevatorMotionAcceleration));
    // elevator.configMotionCruiseVelocity(SmartDashboard.getNumber("Elevator Cruise Vel", ElevatorConstants.kElevatorCruiseVelocity));
    double ff = -ElevatorConstants.kArbitraryFF.get() * Math.sin(armAngleSupplier.getAsDouble());

    // if (!hallEffect.get())
    // {
    //   elevator.setSelectedSensorPosition(ElevatorConstants.kElevatorStow);
    // }

    if ((elevator.getSelectedSensorPosition() >= ElevatorConstants.kElevatorStow - 16666 || elevator.getStatorCurrent() >= 45) && targetTicks == 0) { // TODO: Measure elevator lower limit
      elevator.set(ControlMode.PercentOutput, 0);
    } else {
      elevator.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
    }
  }

  public void moveMotionMagic(int targetTicks) {
    setTargetTicks(targetTicks);
    moveMotionMagic();
  }

  public double percentExtended() {
    return Math.abs(elevator.getSelectedSensorPosition() / (ElevatorConstants.kElevatorScoreHighCone));
  }

  public void setTargetTicks(int targetTicks) {
    this.targetTicks = targetTicks;
  }

  public void setPowerZero() {
    elevator.set(ControlMode.PercentOutput, 0);
  }

  public void setBrakeMode() {
    elevator.setNeutralMode(NeutralMode.Brake);
  }

  public void resetEncoder() {
    elevator.setSelectedSensorPosition(0);
  }

  public void resetEncoderStow() {
    elevator.setSelectedSensorPosition(ElevatorConstants.kElevatorStow);
  } 

  public void reportToSmartDashboard(LOG_LEVEL level) {
    switch (level) {
      case OFF:
        break;
      case ALL:
        SmartDashboard.putNumber("Elevator Motor Output", elevator.getMotorOutputPercent());
        SmartDashboard.putNumber("Elevator Current", elevator.getStatorCurrent());
        SmartDashboard.putNumber("Elevator Percent Extended", percentExtended());
      case MEDIUM:
        SmartDashboard.putNumber("Elevator Current Velocity", elevator.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Elevator Target Velocity", elevator.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Elevator Voltage", elevator.getMotorOutputVoltage());
      case MINIMAL:
        SmartDashboard.putNumber("Elevator Current Ticks", elevator.getSelectedSensorPosition());
        SmartDashboard.putNumber("Elevator Target Ticks", targetTicks);
        break;
    }
  }

  public void initShuffleboard(LOG_LEVEL level) {
    if (level == LOG_LEVEL.OFF)  {
      return;
    }
    ShuffleboardTab tab;
    if (level == LOG_LEVEL.MINIMAL) {
      tab = Shuffleboard.getTab("Main");
    } else {
      tab = Shuffleboard.getTab("Elevator");
    }

    switch (level) {
      case OFF:
        break;
      case ALL:
        tab.addNumber("Target Velocity", () -> elevator.getActiveTrajectoryVelocity());
      case MEDIUM:
        tab.addNumber("Current", () -> elevator.getStatorCurrent());
        tab.addNumber("Velocity", () -> elevator.getSelectedSensorVelocity());
        tab.addNumber("Motor Output", () -> elevator.getMotorOutputPercent());
        tab.addNumber("Voltage", elevator::getMotorOutputVoltage);
        tab.addNumber("Percent Extended", () -> percentExtended());
      case MINIMAL:
        tab.addBoolean("At target position", atTargetPosition);
        tab.addNumber("Current Elevator Ticks", () -> elevator.getSelectedSensorPosition());
        tab.addNumber("Target Elevator Ticks", () -> targetTicks);
        break;
    }    
  }
}
