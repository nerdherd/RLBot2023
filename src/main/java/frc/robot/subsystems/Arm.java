package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase{
    private TalonFX rotatingArm;
    private int targetTicks = ArmConstants.kArmStow;
    public BooleanSupplier atTargetPosition;
    private DigitalInput talonTach;
    private boolean inTalonTachZone;

    public Arm() {
        talonTach = new DigitalInput(ArmConstants.kTalonTachID);
        rotatingArm = new TalonFX(ArmConstants.kRotatingArmID);
    }

    public void init() {
        rotatingArm.setNeutralMode(NeutralMode.Brake);
        rotatingArm.setInverted(false);
        //test
        rotatingArm.config_kP(0, ArmConstants.kArmP);
        rotatingArm.config_kI(0, ArmConstants.kArmI);
        rotatingArm.config_kD(0, ArmConstants.kArmD);
        rotatingArm.config_kF(0, ArmConstants.kArmF);

        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);


    }

    public void moveArmJoystick(double currentJoystickOutput) {
        if (currentJoystickOutput > ArmConstants.kArmDeadband) {
            if (rotatingArm.getStatorCurrent() >= 45)
            {
                rotatingArm.set(ControlMode.PercentOutput, 0);
            } else {
                rotatingArm.set(ControlMode.PercentOutput, 0.3);
            }
        }

        else if (currentJoystickOutput < -ArmConstants.kArmDeadband){
            if (rotatingArm.getStatorCurrent() >= 45) {
                rotatingArm.set(ControlMode.PercentOutput, 0);
            }
            else {
                rotatingArm.set(ControlMode.PercentOutput, -0.3);
            }}

        else {
            rotatingArm.set(ControlMode.PercentOutput, 0);
            rotatingArm.setNeutralMode(NeutralMode.Brake);
        }
    }
    
    public void moveArmMotionMagicButton(int position) {
        rotatingArm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity);
        rotatingArm.configMotionAcceleration(ArmConstants.kArmMotionAcceleration);
        setTargetTicks(position);
    }

    public void setTargetTicks(int targetTicks) {
        this.targetTicks = targetTicks;
    }
    
}
