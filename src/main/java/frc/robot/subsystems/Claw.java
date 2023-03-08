package frc.robot.subsystems;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Claw extends SubsystemBase
{
    Solenoid m_ClawSolenoid;
    static Claw m_Instance;
    Claw()
    {
        m_ClawSolenoid=new Solenoid(PneumaticsModuleType.CTREPCM,ClawConstants.m_ClawSolenoidPort);
    }
    public static Claw GetInstance()
    {
        if(m_Instance==null)
        {
            m_Instance=new Claw();
            return m_Instance;        
        }
        else
        {
            return m_Instance;
        }
    }
    public void Switch () {
        if(m_ClawSolenoid.get())
            Close();
        else
            Ease();
    }
    /**
     * 让夹子松开
     */
    public void Ease()
    {
        m_ClawSolenoid.set(true);
    }
    public void Close()
    {
        m_ClawSolenoid.set(false);
    }
    public Object Open() {
        return null;
    }
    @Override
    public void periodic()
    {
        
    SmartDashboard.putBoolean("clawStatus", m_ClawSolenoid.get());
    }
} 