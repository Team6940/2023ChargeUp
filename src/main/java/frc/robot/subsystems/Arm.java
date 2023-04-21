package frc.robot.subsystems;

import java.time.OffsetDateTime;
import java.util.concurrent.Delayed;

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
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClawConstants;

public class Arm extends SubsystemBase{
    public static Arm instance=null;
    public static boolean m_ArmEnabled=false;
    private TalonFX m_ArmMotorUp;
    private TalonFX m_ArmMotorDown;
    private double NowDegree=0;
    private double TargetAngle=0;
    private ArmFeedforward m_ArmInFeedforward=new ArmFeedforward(ArmConstants.ArmInFeedForwardkS,ArmConstants.ArmInFeedForwardkG,ArmConstants.ArmInFeedForwardkV,ArmConstants.ArmInFeedForwardkA);
    private ArmFeedforward m_ArmOutFeedforward=new ArmFeedforward(ArmConstants.ArmOutFeedForwardkS,ArmConstants.ArmOutFeedForwardkG,ArmConstants.ArmOutFeedForwardkV,ArmConstants.ArmInFeedForwardkA);
    private PIDController m_ArmMotorInPIDCOntroller=new PIDController(ArmConstants.ArmMotorInkP,ArmConstants.ArmMotorInkI,ArmConstants.ArmMotorInkD);
    private PIDController m_ArmMotorOutPIDCOntroller=new PIDController(ArmConstants.ArmMotorOutkP,ArmConstants.ArmMotorOutkI,ArmConstants.ArmMotorOutkD);
    private SlewRateLimiter m_ArmInSlewRateLimiter=new SlewRateLimiter(4);
    private SlewRateLimiter m_ArmOutSlewRateLimiter=new SlewRateLimiter(4);
    private  Solenoid m_ArmSolenoid;
    private  boolean m_ArmSolenoidStatus=false;
    public double offset=0;
    public Arm()
    {
        m_ArmMotorUp=new TalonFX(ArmConstants.ArmMotorUpDeviceNumber);
        m_ArmMotorUp.setInverted(true);
        m_ArmSolenoid =new Solenoid(PneumaticsModuleType.CTREPCM,ArmConstants.ArmSolenoidPort);
        m_ArmMotorUp.config_kP(0, ArmConstants.ArmMotorInkP);
        m_ArmMotorUp.config_kI(0, ArmConstants.ArmMotorInkI);
        m_ArmMotorUp.config_kD(0, ArmConstants.ArmMotorInkD);
        m_ArmMotorUp.configPeakOutputForward(0.27);
        m_ArmMotorUp.configPeakOutputReverse(-0.27);
        m_ArmMotorUp.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        
        m_ArmMotorDown=new TalonFX(ArmConstants.ArmMotorDownDeviceNumber);
        
        m_ArmMotorDown.setInverted(true);
        m_ArmMotorDown.config_kP(0, ArmConstants.ArmMotorOutkP);
        m_ArmMotorDown.config_kI(0, ArmConstants.ArmMotorOutkI);
        m_ArmMotorDown.config_kD(0, ArmConstants.ArmMotorOutkD);
        m_ArmMotorDown.configPeakOutputForward(0.27);
        m_ArmMotorDown.configPeakOutputReverse(-0.27);
        m_ArmMotorDown.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        SpinTo(12);
        // SwitchSolenoidStatus();
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
        if(m_ArmSolenoidStatus==false)
            _NowDegree+=ArmConstants.ArmInDegreeToVerticalOffset;
        else
            _NowDegree+=ArmConstants.ArmOutDegreeToVerticalOffset;
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
        m_ArmMotorInPIDCOntroller.setSetpoint(_Degree);
        m_ArmMotorOutPIDCOntroller.setSetpoint(_Degree);
        TargetAngle=_Degree;
        m_ArmMotorInPIDCOntroller.reset();
        m_ArmMotorOutPIDCOntroller.reset();
    }
    public void SetState(double _Degree,boolean ArmState)
    {
        if(ArmState!=m_ArmSolenoidStatus)
            SwitchSolenoidStatus();
        SpinTo(_Degree);
        
    }
    public void SpinPositive()
    {
        m_ArmMotorUp.set(ControlMode.PercentOutput, 0.07);
        m_ArmMotorDown.set(ControlMode.PercentOutput, 0.07);
    }
    public void Stop()
    {
        
        m_ArmMotorInPIDCOntroller.setSetpoint(GetNowDegree());
        m_ArmMotorOutPIDCOntroller.setSetpoint(GetNowDegree());
    }
    public void SpinNegetive()
    {
        m_ArmMotorUp.set(ControlMode.PercentOutput, -0.2);
        m_ArmMotorDown.set(ControlMode.PercentOutput, -0.2);
    }
    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        NowDegree=GetNowDegree();
        SmartDashboard.putNumber("ArmDegree", NowDegree);
        SmartDashboard.putNumber("MotorPosition", m_ArmMotorUp.getSelectedSensorPosition());
        SmartDashboard.putBoolean("Status",m_ArmSolenoidStatus);
        if(m_ArmEnabled)
        {
            double _ArmMotorOutput=0;
        if(m_ArmSolenoidStatus==false)
        {
           _ArmMotorOutput=m_ArmInFeedforward.calculate(TargetAngle/180*Math.PI, 0)+m_ArmMotorInPIDCOntroller.calculate(NowDegree);
            _ArmMotorOutput=m_ArmInSlewRateLimiter.calculate(_ArmMotorOutput);
            m_ArmOutSlewRateLimiter.calculate(_ArmMotorOutput);
        }
        else
        {
           _ArmMotorOutput=m_ArmOutFeedforward.calculate(TargetAngle/180*Math.PI, 0)+m_ArmMotorOutPIDCOntroller.calculate(NowDegree);
            _ArmMotorOutput=m_ArmOutSlewRateLimiter.calculate(_ArmMotorOutput);
            m_ArmInSlewRateLimiter.calculate(_ArmMotorOutput);
        }
        _ArmMotorOutput+=offset;
        double _ArmMotorDownOutput=m_ArmMotorOutPIDCOntroller.calculate(GetNowDegree());
        m_ArmMotorUp.set(ControlMode.PercentOutput,_ArmMotorOutput);
        m_ArmMotorDown.set(ControlMode.PercentOutput,_ArmMotorOutput);

        }
        }
}
