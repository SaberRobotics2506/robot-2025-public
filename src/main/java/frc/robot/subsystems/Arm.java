package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
	
    private TalonFX m_armMotor;
    private TalonFX m_armMotor2;
    private DutyCycleEncoder arm_encoder;
    public DigitalInput bottomLimit = new DigitalInput(1);
    public DigitalInput topLimit = new DigitalInput(6);

    public Arm(TalonFX m_armMotor, TalonFX m_armMotor2, DutyCycleEncoder m_encoder) {
        this.m_armMotor = m_armMotor;
        this.m_armMotor2 = m_armMotor2;
        this.arm_encoder = m_encoder;
    }

}
