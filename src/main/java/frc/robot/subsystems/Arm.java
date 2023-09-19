package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;
import frc.robot.filters.ExponentialSmoothingFilter;
import frc.robot.util.NerdyMath;


public class Arm extends SubsystemBase implements Reportable {
    private TalonFX rotatingArm;
    private int targetTicks = ArmConstants.kArmStow;
    public BooleanSupplier atTargetPosition;
    private DigitalInput talonTach;
    private boolean inTalonTachZone;
    private DoubleSupplier percentExtended; 
    private ExponentialSmoothingFilter smoothingFilter;

    public Arm(DoubleSupplier percentExtended) {
        talonTach = new DigitalInput(ArmConstants.kTalonTachID);
        rotatingArm = new TalonFX(ArmConstants.kRotatingArmID);
        this.percentExtended = percentExtended; 
        atTargetPosition = () -> NerdyMath.inRange(rotatingArm.getSelectedSensorPosition(), targetTicks - 1500, targetTicks + 1500);
    }

    public void init() {
        rotatingArm.setNeutralMode(NeutralMode.Brake);
        rotatingArm.setInverted(false);
        //test
        rotatingArm.config_kP(0, ArmConstants.kArmP.get());
        rotatingArm.config_kI(0, ArmConstants.kArmI.get());
        rotatingArm.config_kD(0, ArmConstants.kArmD.get());
        rotatingArm.config_kF(0, ArmConstants.kArmF.get());

        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);
    }

    @Override
    public void periodic() {
        moveArmMotionMagic();
    }

    // public void moveArmJoystick(double currentJoystickOutput) {
    //     if (currentJoystickOutput > ArmConstants.kArmDeadband) {
    //         if (rotatingArm.getStatorCurrent() >= 45)
    //         {
    //             rotatingArm.set(ControlMode.PercentOutput, 0);
    //         } else {
    //             rotatingArm.set(ControlMode.PercentOutput, 0.3);
    //         }
    //     }

    //     else if (currentJoystickOutput < -ArmConstants.kArmDeadband){
    //         if (rotatingArm.getStatorCurrent() >= 45) {
    //             rotatingArm.set(ControlMode.PercentOutput, 0);
    //         }
    //         else {
    //             rotatingArm.set(ControlMode.PercentOutput, -0.3);
    //         }}

    //     else {
    //         rotatingArm.set(ControlMode.PercentOutput, 0);
    //         rotatingArm.setNeutralMode(NeutralMode.Brake);
    //     }
    // }

    public void moveArmMotionMagicButton(int position) {
        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);
        setTargetTicks(position);
    }

    public void moveArmMotionMagic() {
        double ff = -(ArmConstants.kStowedFF.get() + ArmConstants.kDiffFF.get() * percentExtended.getAsDouble()) * Math.cos(getArmAngle());
        rotatingArm.set(ControlMode.MotionMagic, targetTicks, DemandType.ArbitraryFeedForward, ff);
    }

    public void moveArmMotionMagicJoystick(double joystickInput) {
        if (joystickInput > 0.05 || joystickInput < -0.05) {
            targetTicks += smoothingFilter.calculate(joystickInput) * ArmConstants.kArmManualTicksPer20ms;
            setTargetTicks(targetTicks);
        } else {
            smoothingFilter.reset();
        }
    }

    public void setTargetTicks(int targetTicks) {
        this.targetTicks = targetTicks;
    }
    
    public void setArmBrakeMode() {
        rotatingArm.setNeutralMode(NeutralMode.Brake);
    }
    
    public double getArmAngle() {
        return Math.toRadians(Math.abs(rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF)  {
            return;
        }
        ShuffleboardTab tab;
        if (level == LOG_LEVEL.MINIMAL) {
            tab = Shuffleboard.getTab("Main");
        } else {
            tab = Shuffleboard.getTab("Arm");
        }
        switch (level) {
            case OFF:
                break;
            case ALL:
                tab.addNumber("Motor Output", rotatingArm::getMotorOutputPercent);
                tab.addString("Control Mode", rotatingArm.getControlMode()::toString);
                tab.addNumber("target velocity", rotatingArm::getActiveTrajectoryVelocity);
                tab.addNumber("arm target velocity", rotatingArm::getActiveTrajectoryVelocity);
                tab.addNumber("Closed loop error", rotatingArm::getClosedLoopError);
            case MEDIUM:
                tab.addNumber("Arm Current", rotatingArm::getStatorCurrent);
                tab.addNumber("Arm Velocity", rotatingArm::getSelectedSensorVelocity);
                tab.addNumber("Arm Voltage", rotatingArm::getMotorOutputVoltage);
                tab.addNumber("Arm Percent Output", rotatingArm::getMotorOutputPercent);
                tab.addNumber("Angle", () -> (rotatingArm.getSelectedSensorPosition()) / ArmConstants.kTicksPerAngle);
            case MINIMAL:
                tab.addNumber("Current Arm Ticks", rotatingArm::getSelectedSensorPosition);
                tab.addNumber("Target Arm Ticks", () -> targetTicks);
                tab.addBoolean("At target position", atTargetPosition);
                break;
        }

        
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
                // SmartDashboard.putBoolean("Limit switch", limitSwitch.get());
        
                SmartDashboard.putNumber("Arm Motor Output", rotatingArm.getMotorOutputPercent());
                SmartDashboard.putNumber("Arm Angle", Math.toDegrees(getArmAngle()));
        
                // SmartDashboard.putString("Arm Control Mode", rotatingArm.getControlMode().toString());
                // SmartDashboard.putNumber("Closed Loop Target", rotatingArm.getClosedLoopTarget());
                // SmartDashboard.putNumber("arm target velocity", rotatingArm.getActiveTrajectoryVelocity());
                SmartDashboard.putNumber("arm velocity", rotatingArm.getSelectedSensorVelocity());
                // SmartDashboard.putNumber("Closed loop error", rotatingArm.getClosedLoopError());
                // if (this.getCurrentCommand() != null) {
                //     SmartDashboard.putBoolean("Arm subsystem", this.getCurrentCommand() == this.getDefaultCommand());
                // }
            case MEDIUM:
                SmartDashboard.putNumber("Arm Current", rotatingArm.getStatorCurrent());
                SmartDashboard.putNumber("Arm Voltage", rotatingArm.getMotorOutputVoltage());
            case MINIMAL:
                SmartDashboard.putNumber("Arm Ticks", rotatingArm.getSelectedSensorPosition());
                SmartDashboard.putNumber("Target Arm Ticks", targetTicks);
                break;
        }
    }
}
