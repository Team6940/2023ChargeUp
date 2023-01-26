package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
/**
 * SwerveModule定义了一个单独的由动轮电机和转向电机组成的Swerve矢量底盘模块，你可以从这里读到两个电机的各项输出，同时设定这个Swerve模块向哪个方向以什么速度移动。
 */
public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final WPI_TalonFX m_DriveMotor;
  private final WPI_TalonSRX m_PivotMotor;
  private double m_LastAngle;
  
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

  private int m_Offset;

  private double m_DriveMotorInverted = 1.0;
  private double m_PivotMotorInverted = 1.0;
    
  private boolean m_DriveMotorOutputEnabled = true;
  private boolean m_PivotMotorOutputEnabled = true;
  private double m_PivotEncoderInverted = 1.0;
  /**
   * 初始化一个swerve模块
   * @param _DriveDeviceNumber 动轮模块的端口编号
   * @param _PivotDeviceNumber 转向轮模块的端口编号
   * @param _DriveMotorInvert  动轮模块是否反转
   * @param _PivotMotorInvert  转向轮模块是否反转
   * @param _PivotEncoderOffset 转向轮编码器偏移量
   * @param _PivotEncoderPhase ？不知道这个变量有什么用
   * @param _PivotEncoderInvert 转向轮编码器是否反转
   */
  public SwerveModule(int _DriveDeviceNumber, int _PivotDeviceNumber,
                      boolean _DriveMotorInvert, boolean _PivotMotorInvert,
                      int _PivotEncoderOffset, boolean _PivotEncoderPhase,
                      boolean _PivotEncoderInvert) {//这是SwerveModule的构造函数
    m_DriveMotor = new WPI_TalonFX(_DriveDeviceNumber);//设定这个Swerve模块的动轮电机编号
    m_PivotMotor = new WPI_TalonSRX(_PivotDeviceNumber); //设定这个Swerve模块的动轮电机模块

    //These two may let the swerve rotate itself many times when startup
    //drive_motor_.configFactoryDefault();
    //pivot_motor_.configFactoryDefault();
    
    m_DriveMotor.setNeutralMode(NeutralMode.Brake);//不知道为什么这么做，反正照做就是了
    m_PivotMotor.setNeutralMode(NeutralMode.Coast);//不知道为什么这么做，反正照做就是了
    m_DriveMotor.configPeakOutputForward( SwerveConstants.kDriveMotorMaxOutput);//设定动轮电机最高输出
    m_DriveMotor.configPeakOutputReverse(-SwerveConstants.kDriveMotorMaxOutput);//设定动轮电机反向最高输出
    m_PivotMotor.configPeakOutputForward( SwerveConstants.kPivotMotorMaxOutput);//设定转轮电机最高输出
    m_PivotMotor.configPeakOutputReverse(-SwerveConstants.kPivotMotorMaxOutput);//设定转轮电机反向最高输出

    m_DriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);//设定动轮电机控制器用来测量电机转速的传感器
    m_PivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);//设定转轮电机控制器用来测量电机转速的传感器

    m_DriveMotor.config_kP(0, SwerveConstants.kDriveMotorkP);//设定动轮电机的P值
    m_DriveMotor.config_kI(0, SwerveConstants.kDriveMotorkI);//设定动轮电机的I值
    m_DriveMotor.config_kD(0, SwerveConstants.kDriveMotorkD);//设定动轮电机的D值
    m_DriveMotor.config_kF(0, SwerveConstants.kDriveMotorkF);//设定动轮电机的F值
    m_DriveMotor.config_IntegralZone(0, SwerveConstants.kDriveMotorIZone);//设定动轮电机的最大积分范围
  
    m_PivotMotor.config_kP(0, SwerveConstants.kPivotMotorkP);//设定转轮电机的P值
    m_PivotMotor.config_kI(0, SwerveConstants.kPivotMotorkI);//设定转轮电机的I值
    m_PivotMotor.config_kD(0, SwerveConstants.kPivotMotorkD);//设定转轮电机的D值
    m_PivotMotor.config_kF(0, SwerveConstants.kPivotMotorF);//设定转轮电机的F值
    m_PivotMotor.config_IntegralZone(0, SwerveConstants.kPivotMotorkIZone);//设定转轮电机的最大积分范围
    m_PivotMotor.configMotionCruiseVelocity(SwerveConstants.motionCruiseVelocity);//设定转轮电机的最大速度
    m_PivotMotor.configMotionAcceleration(SwerveConstants.motionAcceleration);//设定转轮电机的最大加速度

    m_DriveMotor.configOpenloopRamp(SwerveConstants.kLoopSeconds);//不知道有什么用
    m_DriveMotor.configClosedloopRamp(SwerveConstants.kLoopSeconds);//不知道有什么用

    m_PivotMotor.configOpenloopRamp(SwerveConstants.kLoopSeconds);//不知道有什么用
    m_PivotMotor.configClosedloopRamp(SwerveConstants.kLoopSeconds);//不知道有什么用

    m_PivotEncoderInverted = _PivotEncoderInvert ? -1.0 : 1.0;//设定电机编码器是否反转

    m_DriveMotor.configVoltageCompSaturation(12);//反正是设定标准参数，不知道有什么用，照抄就是了
    m_DriveMotor.enableVoltageCompensation(true);//反正是设定标准参数，不知道有什么用，照抄就是了

    m_PivotMotor.configVoltageCompSaturation(12);//反正是设定标准参数，不知道有什么用，照抄就是了
    m_PivotMotor.enableVoltageCompensation(true);//反正是设定标准参数，不知道有什么用，照抄就是了

    m_DriveMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms);
    m_DriveMotor.configVelocityMeasurementWindow(1);

    // Sets current limits for motors
    //drive_motor_.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
    //SwerveConstants.SWERVE_MOTOR_CURRENT_LIMIT, SwerveConstants.SWERVE_MOTOR_CURRENT_LIMIT, 0));

    //pivot_motor_.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
    //SwerveConstants.SWERVE_DRIVE_MOTOR_CURRENT_LIMIT, SwerveConstants.SWERVE_DRIVE_MOTOR_CURRENT_LIMIT, 0));

    //drive_motor_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
    //drive_motor_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    //pivot_motor_.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 50);
    //pivot_motor_.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);

    SetDriveMotorInverted(_DriveMotorInvert);//设定动轮电机是否反转
    SetPivotMotorInverted(_PivotMotorInvert);//设定转轮电机是否反转
    SetPivotEncoderOffset((int)m_PivotEncoderInverted * _PivotEncoderOffset);//设定转轮电机编码器的偏移量
    SetPivotEncoderPhase(_PivotEncoderPhase);//不知道有什么用

    m_LastAngle = GetState().angle.getRadians();
  }
  /**
   * 设定动轮电机是否反转
   * @param _Invert 动轮电机是否反转
   */
  public void SetDriveMotorInverted(boolean _Invert){
    m_DriveMotorInverted = _Invert ? -1.0 : 1.0;
  }
  /**
   * 设定转轮电机是否反转
   * @param _Invert 转轮电机是否反转
   */
  public void SetPivotMotorInverted(boolean _Invert){
    m_PivotMotorInverted = _Invert ? -1.0 : 1.0;
    // TODO(Shimushu): This doesn't work as I expected
    // pivot_motor_.SetInverted(invert);
    // TODO(Shimushu): I don't know this works accidentally or not
    m_PivotMotor.setSensorPhase(!_Invert);
  }
  /**
   * 设定转轮偏移量
   * @param _Offset 转轮偏移量
   */
  public void SetPivotEncoderOffset(int _Offset){
    m_Offset = _Offset;
  }
  /**
   * 设定动轮编码器相位
   * @param _Phase 动轮编码器相位
   */
  public void SetPivotEncoderPhase(boolean _Phase){
    m_PivotMotor.setSensorPhase(_Phase);
  }
  /**
   * 设定两个模块的两个是否disable
   * @param _Disable 是否disable两个电机
   */
  public void SetMotorOutputDisabled(boolean _Disable){
    m_DriveMotorOutputEnabled = !_Disable;
    m_PivotMotorOutputEnabled = !_Disable;
  }
/**
 * 设定动轮电机是否输出到电机上
 * @param _Disable 是否输出到电机上
 */
  public void SetDriveMotorOutputDisabled(boolean _Disable){
    
    m_DriveMotorOutputEnabled = !_Disable;
  }
  /**
   * 设定转轮输出是否输出到电机上
   * @param _Disable 是否输出到电机上
   */
  public void SetPivotMotorOutputDisabled(boolean _Disable){
    m_PivotMotorOutputEnabled = !_Disable;
  }
  /**
   * @return 返回动轮电机的输出功率
   */
  public double GetDriveMotorOutput(){
    return m_DriveMotor.getMotorOutputPercent();
  }

  /**
   * @return 返回转轮电机的输出功率
   */
  public double GetPivotMotorOutput(){
    return m_PivotMotor.getMotorOutputPercent();
  }
  /**
   * @return 以动轮速度+转轮朝向的形式返回swerve模块的状态
   */
  public SwerveModuleState GetState(){
    
    
    return new SwerveModuleState(
      GetSpeed(),
      new Rotation2d(GetAngle())
    );
  }
  
  public SwerveModulePosition GetPosition()
  {
    return new SwerveModulePosition(
      GetDistanceMeters(),
      new Rotation2d(GetAngle())
    );
  }
  /**
   * 设定swerve模块的目标状态 
   * @param state 要设定的Swerve模块状态
   * @param isOpenLoop 无意义参数，默认true
   */
  public void SetDesiredState(SwerveModuleState state,boolean isOpenLoop){
    double rawAngle = GetRawAngle();
    double angle = MathUtil.angleModulus(rawAngle);

    if(Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeed * 0.05))//除去摇杆的微扰，过于小的位移需求不去处理
    {
      state.speedMetersPerSecond = 0;
    }
    /**
     * 这里使用了optimize函数优化state使得转轮尽可能少地旋转
     * 例如：目前转轮朝向的是0°，想要设定的state是(100,190°)，
     * optimize会把其转化为(-100,10°)，大概是这样的简化函数。
     */
    SwerveModuleState optimalState = SwerveModuleState.optimize(//
      state, new Rotation2d(angle));

    double driveOutput = optimalState.speedMetersPerSecond / //计算动轮的输出速度
    (SwerveConstants.kWheelDiameter / 2.0) /
    SwerveConstants.kDriveEncoderReductionRatio *
    SwerveConstants.kDriveEncoderResolution * 0.1;

    //final double CTREDriveOutput = Conversions.MPSToFalcon(optimalState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);

    double optimalAngle = MathUtil.angleModulus(optimalState.angle.getRadians());//计算转轮的输出角度
    //如果转轮的输出角度太低了就不转了
    optimalAngle += rawAngle - angle;
    if(Math.abs(rawAngle - optimalAngle) >= Math.PI * 0.5)
    {
      optimalAngle += Math.copySign(Math.PI * 2, 
                      rawAngle - optimalAngle);
    }

    //如果动轮的转速太低了就不转了
    double Angle = (Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeed * 0.01)) ? m_LastAngle : optimalAngle;
    
    double pivotOutput = Angle /
    SwerveConstants.kPivotEncoderReductionRatio *
    SwerveConstants.kPivotEncoderResolution *
    m_PivotEncoderInverted + m_Offset;

    //double MaxFreeSpeed = Conversions.RPMToFalcon(6380, 1);

    //double RealFreeSpeed = 6380 / 60 * Constants.SwerveConstants.driveGearRatio * Constants.kWheelDiameter * Math.PI;
    //SmartDashboard.putNumber("FreeRealSpeed", RealFreeSpeed);

    double percentOutput = feedforward.calculate(optimalState.speedMetersPerSecond);

    SmartDashboard.putNumber("OptimalSpeed", optimalState.speedMetersPerSecond);
    SmartDashboard.putNumber("Debug/Drive/PercentOut", percentOutput);
    //以下为将动轮和转轮输出到电机
    if(isOpenLoop)
    {
      if (m_DriveMotorOutputEnabled) 
      {
        //drive_motor_.set(ControlMode.PercentOutput,
        //    drive_motor_inverted * percentOutput);
        //将
        m_DriveMotor.set(ControlMode.Velocity, //TODO: Test combining FeedForward and velocity closed loop
            m_DriveMotorInverted * driveOutput/*, DemandType.ArbitraryFeedForward,
            drive_motor_inverted * percentOutput*/);
      }
    }
    else
    {
      if (m_DriveMotorOutputEnabled) 
      {
        m_DriveMotor.set(ControlMode.Velocity,
            m_DriveMotorInverted * driveOutput);
        /* Try combing PID Control with FeedForward */
        
        //drive_motor_.set(ControlMode.Velocity, //TODO: Test combining FeedForward and velocity closed loop
        //    drive_motor_inverted * driveOutput, DemandType.ArbitraryFeedForward,
        //    drive_motor_inverted * percentOutput);
      }
    }
    if (m_PivotMotorOutputEnabled) 
    {
      m_PivotMotor.set(ControlMode.MotionMagic,
          m_PivotMotorInverted * pivotOutput);
    }
    m_LastAngle = Angle;
  }
  /**
   * 获取动轮的电压
   * @return 动轮的电压
   */
  public double GetDriveMotorVoltage(){
    /*The unit is volt*/
    return m_DriveMotor.getBusVoltage();
  }
  /**
   * 获取转轮的电压
   * @return 转轮的电压（单位为伏）
   */
  public double GetTurnMotorVoltage(){
    /*The unit is volt*/
    return m_PivotMotor.getBusVoltage();
  }
  /**
   * 获取动轮的输出电压
   * @return 动轮的电压（单位为伏）
   */
  public double GetDriveMotorOutputVoltage(){
    /*The unit is volt*/
    return m_DriveMotor.getMotorOutputVoltage();
  }
/**
 * 获取转轮的输出电压
 * @return 转轮的输出电压（单位为伏）
 */
  public double GetTurnMotorOutputVoltage(){
    /*The unit is volt*/
    return m_PivotMotor.getMotorOutputVoltage();
  }
  /**
   * 获取动轮的输出电流
   * @return 动轮的输出电流（单位为安）
   */
  public double GetDriveMotorCurrent(){
    /*The unit is ampere*/
    return m_DriveMotor.getSupplyCurrent();
  }
  /**
   * 获取转轮的输出电流
   * @return 转轮的输出电流（单位为安）
   */
  public double GetTurnMotorCurrent(){
    /*The unit is ampere*/
    return m_PivotMotor.getSupplyCurrent();
  }
  /**
   * 获取动轮的输出功率
   * @return 动轮的输出功率（单位为瓦）
   */
  public double GetDriveMotorPower(){
    /*The unit is watt*/
    return GetDriveMotorVoltage() * GetDriveMotorCurrent();
  }
  /**
   * 获取转轮的输出功率
   * @return 转轮的输出功率（单位为瓦）
   */
  public double GetTurnMotorPower(){
    /*The unit is watt*/
    return GetTurnMotorVoltage() * GetTurnMotorCurrent();
  }
  /**
   * 获取该swerve模块占用的总电流，但是这没有意义啊？
   * @return swerve模块占用的总电流（单位为安）
   */
  public double GetTotalCurrent(){
    /*The unit is ampere*/
    return GetDriveMotorCurrent() + GetTurnMotorCurrent();
  }
  /**
   * 获取该swerve模块占用的总功率
   * @return swerve模块占用的总功率单位为瓦
   */
  public double GetTotalPower(){
    /*The unit is watt*/
    return GetDriveMotorPower() + GetTurnMotorPower();
  }
  /**
   * 获取动轮电机的温度
   * @return 动轮电机的温度（单位为摄氏度）
   */
  public double GetDriveMotorTemperature(){
    /*The unit is celsius */
    return m_DriveMotor.getTemperature();
  }
  /**
   * 获取转轮电机的温度
   * @return 转轮电机的温度（单位为摄氏度）
   */
  public double GetPivotMotorTemperature(){
    /*The unit is celsius */
    return m_PivotMotor.getTemperature();
  }
  /**
   * 获取转轮电机的当前朝向，该弧度数值经过处理使得范围处于[-pi,pi]之间
   * @return 转轮电机的当前朝向(单位为rad,范围为[-pi,pi])
   */
  public double GetAngle(){
    /*The unit is radian*/
    return MathUtil.angleModulus(      
    (m_PivotMotor.getSelectedSensorPosition() - m_Offset) /
    SwerveConstants.kPivotEncoderResolution *
    SwerveConstants.kPivotEncoderReductionRatio *
    m_PivotEncoderInverted);
  }
  /**
   * 获取转轮电机的当前朝向，不经处理的数值
   * @return 转轮电机的当前朝向(单位为rad,范围为R)
   */
  public double GetRawAngle(){
    /*The unit is radian*/
    return 
        (m_PivotMotor.getSelectedSensorPosition() - m_Offset) /
        SwerveConstants.kPivotEncoderResolution *
        SwerveConstants.kPivotEncoderReductionRatio *
        m_PivotEncoderInverted;
  }
  /**
   * 获取动轮的速度，单位为m/s
   * @return 动轮的速度，单位为m/s
   */
  public double GetSpeed(){
    /*The unit is meters_per_second */
    return
        m_DriveMotor.getSelectedSensorVelocity() * m_DriveMotorInverted / 0.1 /
        SwerveConstants.kDriveEncoderResolution *
        SwerveConstants.kDriveEncoderReductionRatio *
            SwerveConstants.kWheelDiameter / 2;
    //return Conversions.falconToMPS(drive_motor_.getSelectedSensorVelocity(), Constants.kWheelDiameter * Math.PI,
    //    Constants.kDriveEncoderReductionRatioTest);
  }
  /**
   * 输出这个的模块从启动到现在一共移动了多少米。主要是用来应对WPILib2023中把里程表的传入参数从velocity改成Distance的改变
   * @return 模块移动的距离，单位为米
   */
  public double GetDistanceMeters()
  {
    return
    m_DriveMotor.getSelectedSensorPosition() * m_DriveMotorInverted /
    SwerveConstants.kDriveEncoderResolution *
    SwerveConstants.kDriveEncoderReductionRatio *
        SwerveConstants.kWheelDiameter / 2;
  }
}
