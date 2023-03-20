package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.LayDownPreventionConstants;

public class DownPrevention extends SubsystemBase {
    
    Solenoid m_DownPreventSolenoid;
    static DownPrevention m_Instance;
    DownPrevention()
    {
        m_DownPreventSolenoid=new Solenoid(PneumaticsModuleType.CTREPCM, LayDownPreventionConstants.DownPreventionPort);
    
    }
    public static DownPrevention GetInstance()
    {
        if(m_Instance==null)
        {
            m_Instance=new DownPrevention();
            return m_Instance;        
        }
        else
        {
            return m_Instance;
        }
    }
    public void Switch () {
        if(m_DownPreventSolenoid.get())
            In();
        else
            Out();
    }
    /**
     * 让
     */
    public void Out()
    {
        m_DownPreventSolenoid.set(true);
    }
    public void In()
    {
        m_DownPreventSolenoid.set(false);
    }
    @Override
    public void periodic()
    {
        if(RobotContainer.m_Arm.GetNowDegree()>90&&m_DownPreventSolenoid.get()==false)
            Out();
        if(RobotContainer.m_Arm.GetNowDegree()<90&&m_DownPreventSolenoid.get()==true)
            In();
    SmartDashboard.putBoolean("DownPreventionStatus", m_DownPreventSolenoid.get());
    }
    
}
