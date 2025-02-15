package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extake extends SubsystemBase{

    //initialize motor and IR Sensor
    private TalonSRX m_extakeMotor;
    private DigitalInput irSensor;

    //Constructor
    public Extake(TalonSRX m_extakeMotor, DigitalInput irSensor) {
        this.m_extakeMotor = m_extakeMotor;
        this.irSensor = irSensor;
    }

    //Runs the extake motor
    public void runMotor() {
        m_extakeMotor.set(ControlMode.PercentOutput, Constants.EXTAKE_MOTOR_SPEED);
    }

    public void runMotorReverse() {
        m_extakeMotor.set(ControlMode.PercentOutput, -0.4);
    }

    //Stops the extake motor
    public void stopMotor() {
        m_extakeMotor.set(ControlMode.PercentOutput, 0);
    }

    //returns the value of the IR Sensor
    public boolean getIrSensor(){
        return irSensor.get();
    }

    //gets the velocity of the extake motor
    // public double getMotorVelocity() {
    //     return m_extakeMotor.getVelocity().getValueAsDouble();
    // }
}

