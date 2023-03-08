package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;

public class Arm extends SubsystemBase{
    public static Arm instance=null;
    private TalonFX m_ArmMotorUp;
    private TalonFX m_ArmMotorDown;
    private double NowDegree=0;
    private ArmFeedforward m_ArmFeedforward=new ArmFeedforward(ArmConstants.ArmFeedForwardkS,ArmConstants.ArmFeedForwardkG,ArmConstants.ArmFeedForwardkV,ArmConstants.ArmFeedForwardkA);
    private PIDController m_ArmMotorUpPIDCOntroller=new PIDController(ArmConstants.ArmMotorUpkP,ArmConstants.ArmMotorUpkI,ArmConstants.ArmMotorUpkD);
    private PIDController m_ArmMotorDownPIDCOntroller=new PIDController(ArmConstants.ArmMotorDownkP,ArmConstants.ArmMotorDownkI,ArmConstants.ArmMotorDownkD);
    private  Solenoid m_ArmSolenoid;
    private  boolean m_ArmSolenoidStatus=false;
    public Arm()
    {
        m_ArmMotorUp=new TalonFX(ArmConstants.ArmMotorUpDeviceNumber);
        m_ArmMotorUp.setInverted(true);
       m_ArmSolenoid =new Solenoid(PneumaticsModuleType.CTREPCM,ArmConstants.ArmSolenoidPort);
        m_ArmMotorUp.config_kP(0, ArmConstants.ArmMotorUpkP);
        m_ArmMotorUp.config_kI(0, ArmConstants.ArmMotorUpkI);
        m_ArmMotorUp.config_kD(0, ArmConstants.ArmMotorUpkD);
        m_ArmMotorUp.configPeakOutputForward(0.5);
        m_ArmMotorUp.configPeakOutputReverse(-0.5);
        m_ArmMotorUp.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        m_ArmMotorDown=new TalonFX(ArmConstants.ArmMotorDownDeviceNumber);
        
        m_ArmMotorDown.setInverted(true);
        m_ArmMotorDown.config_kP(0, ArmConstants.ArmMotorDownkP);
        m_ArmMotorDown.config_kI(0, ArmConstants.ArmMotorDownkI);
        m_ArmMotorDown.config_kD(0, ArmConstants.ArmMotorDownkD);
        m_ArmMotorDown.configPeakOutputForward(0.3);
        m_ArmMotorDown.configPeakOutputReverse(-0.3);
        m_ArmMotorDown.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // m_ArmPIDController.setSetpoint(0);   
        // m_SlewRateLimiter.calculate(0);
    }
    public void SwitchSolenoidStatus()
    {
        m_ArmSolenoidStatus=!m_ArmSolenoidStatus;
        m_ArmSolenoid.set(m_ArmSolenoidStatus);
    }
    public static Arm GetInstance()
    {
        if(instance==null)
        {
            instance=new Arm();
            return instance;
        }
        else
            return instance;
    }

    public double GetNowDegree()
    {
        double _NowDegree=(m_ArmMotorUp.getSelectedSensorPosition()-ArmConstants.kArmMotorUpOffeset)*ArmConstants.kArmMotorReductionRatio;
        return _NowDegree;
    }
    /**
     * 让arm以特定转速旋转（未使用PID，采用直接映射）
     * @param _Rct 输出百分比
     */
    public void SpinAt(double _Pct)
    {
        m_ArmMotorUp.set(ControlMode.PercentOutput, _Pct);
        m_ArmMotorDown.set(ControlMode.PercentOutput, _Pct);
    }
    public void SpinTo(double _Degree)
    {
        NowDegree=_Degree;
        m_ArmMotorUpPIDCOntroller.setSetpoint(_Degree);
        m_ArmMotorDownPIDCOntroller.setSetpoint(_Degree);
    }
    public void SpinPositive()
    {
        m_ArmMotorUp.set(ControlMode.PercentOutput, 0.07);
        m_ArmMotorDown.set(ControlMode.PercentOutput, 0.07);
    }
    public void Stop()
    {
        
        m_ArmMotorUpPIDCOntroller.setSetpoint(GetNowDegree());
        m_ArmMotorDownPIDCOntroller.setSetpoint(GetNowDegree());
    }
    public void SpinNegetive()
    {
        m_ArmMotorUp.set(ControlMode.PercentOutput, -0.2);
        m_ArmMotorDown.set(ControlMode.PercentOutput, -0.2);
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        NowDegree=m_ArmMotorUp.getSelectedSensorPosition()*ArmConstants.kArmMotorReductionRatio;
        SmartDashboard.putNumber("ArmDegree", NowDegree);
        SmartDashboard.putNumber("MotorPosition", m_ArmMotorUp.getSelectedSensorPosition());
        double _ArmMotorUpOutdput=m_ArmMotorUpPIDCOntroller.calculate(GetNowDegree());
        double _ArmMotorDownOutput=m_ArmMotorDownPIDCOntroller.calculate(GetNowDegree());
        // m_ArmMotorUp.set(ControlMode.PercentOutput,_ArmMotorUpOutdput);
        // m_ArmMotorDown.set(ControlMode.PercentOutput,_ArmMotorUpOutdput);
    }
}
