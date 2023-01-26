package frc.robot.subsystems;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.library.team1706.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.GlobalConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
/**
 * SwerveBase描述了一个由4个SwerveModule组成的完整的Swerve矢量底盘
 */
public class SwerveBase extends SubsystemBase {

    private static SwerveBase m_Instance = null;
    public static SwerveModuleState[] m_SwerveModuleStates = new SwerveModuleState[4];

    /** Creates a new SwerveDriveTrain. */
    private static SwerveModule m_SwerveModules[] = new SwerveModule[4];//底盘的四个模块
  
    //public PigeonIMU gyro;
    public WPI_PigeonIMU m_Gyro;//陀螺仪
   // public PixyCamSPI mPixy;
    //byte PixySignature;
    
    public boolean Auto = false;
  
    double Preverror;
    double Responsetime = 0.02;
    int i = 1;
    double HEAD_P = 0.01;
    double HEAD_I = 0;
    double HEAD_D = 0;
    PIDController HeadController = new PIDController(HEAD_P, HEAD_I, HEAD_D);
  
    public boolean IsOpenLoop = true;
  
    public boolean WhetherStoreYaw = false;
  
    public boolean AutoPixy = false;
  
    private Field2d m_Field = new Field2d();
  
    private FieldRelativeSpeed m_FieldRelVel = new FieldRelativeSpeed();
    private FieldRelativeSpeed m_LastFieldRelVel = new FieldRelativeSpeed();
    private FieldRelativeAccel m_FieldRelAccel = new FieldRelativeAccel();
  
    private double GyroRollVelocity = 0;
    private double m_LastGyroRoll = 0;
    
  
    ShuffleboardTab m_SwerveDriveTab = Shuffleboard.getTab("Swerve");
    boolean m_EnanbleTelemetry = true;
    
    double m_TargetLockedTime=0;//锁定视觉目标的时间刻，用来判断是否长时间锁定目标的
    boolean m_IsTargetLocked=false;//是否锁定目标，也是用来判断是否长时间锁定目标的，这里用了一个很简单的中断逻辑，不会可以自己去查一下。
    public final static SwerveDriveKinematics m_kDriveKinematics =
        new SwerveDriveKinematics(
          new Translation2d( SwerveConstants.kLength / 2,  SwerveConstants.kWidth / 2),//front left
          new Translation2d( SwerveConstants.kLength / 2, -SwerveConstants.kWidth / 2),//front right
          new Translation2d(-SwerveConstants.kLength / 2,  SwerveConstants.kWidth / 2),//back left
          new Translation2d(-SwerveConstants.kLength / 2, -SwerveConstants.kWidth / 2)//back right
    );
    
    public SwerveDriveOdometry m_Odometry =
        new SwerveDriveOdometry(
          SwerveConstants.swerveKinematics,new Rotation2d(0),
          new SwerveModulePosition[]{new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition(),new SwerveModulePosition()},
          new Pose2d()
    );
  
    public SwerveBase() {
  
      addShuffleboardDebug();
      m_Gyro = new WPI_PigeonIMU(SwerveConstants.PigeonIMUPort);
  
      // The coordinate system may be wrong 
      m_SwerveModules[0] = new SwerveModule(1, 2, false,  false, 2510, true, true);//front left
      m_SwerveModules[1] = new SwerveModule(3, 4, true, false, 3303, true, true);//front right
      m_SwerveModules[2] = new SwerveModule(5, 6, false,  false, 711,  false, false);//back left
      m_SwerveModules[3] = new SwerveModule(7, 8, false, false, 2651,  false, false);//back right
      
      //ahrs = new AHRS(SPI.Port.kMXP);
  
      //mPixy = PixyCamSPI.getInstance();
      /* select cargo color for sig */
      //PixySignature = SmartDashboard.getBoolean("Debug/Pixy/alliance", false) ? Pixy2CCC.CCC_SIG1 : Pixy2CCC.CCC_SIG2;
  
      SmartDashboard.putData("Debug/Drive/Field", m_Field);
    }
    
    public static SwerveBase getInstance() {
      if (m_Instance == null){
        m_Instance = new SwerveBase();
      }
      return m_Instance;
    }
  /**
   * 让swerve底盘动起来
   * @param translation 车子的移动方向
   * @param omega 车子的角速度
   * @param fieldRelative 是否以场地为参考系
   * @param isOpenloop 无意义参数，但是保留
   */
    public void Drive(Translation2d translation,double omega,boolean fieldRelative,boolean isOpenloop){
      var states = SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? 
        ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(), translation.getY(), omega, GetGyroRotation2d())
        : new ChassisSpeeds(translation.getX() , translation.getY(), omega)
      );
  
      SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeed);
        
      for(int i = 0;i < m_SwerveModules.length;i++ ){
        m_SwerveModules[i].SetDesiredState(states[i],isOpenloop);
      }
    }
    public void DriveTo(Pose2d _TargetPose2d){

    }
    public void SetModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeed);
        for(int i = 0;i < m_SwerveModules.length;i++){
          m_SwerveModules[i].SetDesiredState(desiredStates[i], true);
        }  
    }
  
        /**
       * What the module states should be in hold mode. The wheels will be put in an X pattern to prevent the robot from moving.
       * <p>
       * 0 -> Left Front
       * <p>
       * 1 -> Left Back
       * <p>
       * 2 -> Right Front
       * <p>
       * 3 -> Right Back
       */
      public static final SwerveModuleState[] HOLD_MODULE_STATES = {
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
  
    /**
    * What the module states should be set to when we start climbing. All the wheels will face forward to make the robot easy to
    * push once it is disabled.
    * <p>
    * 0 -> Left Front
    * <p>
    * 1 -> Left Back
    * <p>
    * 2 -> Right Front
    * <p>
    * 3 -> Right Back
    */
    public static final SwerveModuleState[] SWERVE_MODULE_STATE_FORWARD = {
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(90))
    };
  
    public void setSWERVE_MODULE_STATE_FORWARD(){
      //SetModuleStates(SWERVE_MODULE_STATE_FORWARD); //TODO
    }
  
    public void setHOLD_MODULE_STATES(){
      SetModuleStates(HOLD_MODULE_STATES);  //TODO
    }
  
      /**
       * 读取基于里程表的车子位置
       *
       * @return The pose.
       */
      public Pose2d getPose() {
        return m_Odometry.getPoseMeters();
      }
  
  /**
   * 读取车子的速度以及角速度
   * @return 车子的速度以及角速度,以chassisSpeed的形式
   */
    public ChassisSpeeds getChassisSpeeds() {
      return m_kDriveKinematics.toChassisSpeeds(
              getStates());
    }
  
    /**
      * 计算每个模块的位移速度并输出到电机上
      * 
      * @param speeds 设定车子的速度
      */
    public void setChassisSpeeds (ChassisSpeeds speeds) {
      SwerveModuleState[] moduleStates = m_kDriveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
      SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);
      SetModuleStates(moduleStates);
    }
    /**
     * 获得以场地为坐标系的车子的速度和角速度
     * @return 车子的速度和角速度，单位为m/s 和rad/s 以chassisSpeed的形式呈现
     */
    public ChassisSpeeds getFieldRelativeChassisSpeeds() {//
      return new ChassisSpeeds(
          getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos() - getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
          getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getCos() + getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getSin(),
          getChassisSpeeds().omegaRadiansPerSecond);
    }
    /**
     * 获得场地坐标系下X轴上的速度
     * @return X轴上的速度，单位m/s
     */
    public double getFieldRelativeXVelocity() {//获得场地坐标系下X轴上的速度
        return getFieldRelativeChassisSpeeds().vxMetersPerSecond;
    }
    /**
     * 获得场地坐标系下y轴上的速度
     * @return y轴上的速度，单位m/s
     */
    public double getFieldRelativeYVelocity() {//获得场地坐标系下y轴上的速度
        return getFieldRelativeChassisSpeeds().vyMetersPerSecond;
    }
    /**
     * 获得场地坐标系下车子的角速度
     * @return 角速度，单位rad/s
     */
    public double getFieldRelativeAngularVelocity() {
        return getFieldRelativeChassisSpeeds().omegaRadiansPerSecond;
    }
  
    /**
     * 获得车子的朝向，以rad为单位
     * @return 车子的朝向，以rad为单位
     */
    public double GetHeading_Rad(){
      /*The unit is radian */
      return GetGyroRotation2d().getRadians();
    }
  /**
     * 获得车子的朝向，以degree为单位
     * @return 车子的朝向，以degree为单位
     */
    public double GetHeading_Deg(){
      /*The unit is radian */
      return GetGyroRotation2d().getDegrees();
    }
    /**
     * 归零车子的朝向
     */
    public void ZeroHeading(){
      WhetherStoreYaw = true;
      // ahrs version
      //ahrs.reset();
      //ResetOdometry(new Pose2d());
      //Pigeon version
      zeroGyro();
    }

    public void TurnOnPixy(){
      AutoPixy = true;
    }
  
    public void TurnOffPixy(){
      AutoPixy = false;
    }
    /**
     * 重置里程表
     * @param pose 将里程表要重置的位置
     */
    public void ResetOdometry(Pose2d pose){
      m_Odometry.resetPosition(GetGyroRotation2d(), GetPositions(),pose);
      //for (int i = 0 ; i < swerve_modules_.length; i++){
      //  swerve_modules_[i].setPose(pose);
      //}
    }
    /**
     * 获取车子的朝向，以rotation2d的形式
     * @return 车子的朝向
     */
    public Rotation2d GetGyroRotation2d(){
      // An offset will be needed if the robot doesn't face downfield
      
      // AHRS version
      //return ahrs.getRotation2d();
  
      //Pigeon Version
      //return Rotation2d.fromDegrees(gyro.getFusedHeading());
      return Rotation2d.fromDegrees(m_Gyro.getRotation2d().getDegrees());
    }
    /**
     * 处理车子的数据，使其忽略微小数值
     * @param x 处理前的数据
     * @return 处理后的数据
     */
    public double deadband(double x){
      return ((x < 0 ? -x : x) < 0.05) ? 0 : x;
    }
    /**
     * 计算车子转向的PID数据
     * @param targetAngle 目标角度
     * @param currentAngle 现在的角度
     * @param kP 无意义参数
     * @param kD 无意义参数
     * @return 头部朝向
     */
    public double calcYawStraight(double targetAngle, double currentAngle,double kP, double kD){
      //The WPILIB's version
      double headadjust = HeadController.calculate(currentAngle, targetAngle);
      return headadjust;
    }
  
    public void resetOdometry(){
      m_Odometry.resetPosition(new Rotation2d(),GetPositions(),new Pose2d());
    }
    /**
     * 获取底盘各个模块的状态
     * @return 各个模块的状态
     */
    public SwerveModuleState[] getStates(){
      SwerveModuleState[] states = new SwerveModuleState[m_SwerveModules.length];
      for (int i = 0;i < m_SwerveModules.length;i++ ){
        states[i] = m_SwerveModules[i].GetState();
      }
      return states;
    }
    public SwerveModulePosition[] GetPositions()
    {
      SwerveModulePosition[] _Positions =new SwerveModulePosition[m_SwerveModules.length];
      for (int i = 0;i < m_SwerveModules.length;i++ ){
        _Positions[i] = m_SwerveModules[i].GetPosition();
      }
      return _Positions;
    }
    /**
     * 重置陀螺仪
     */
    public void zeroGyro(){
      m_Gyro.setFusedHeading(0);
    }
    /**
     * 重置陀螺仪
     * @param reset 将陀螺仪的数值设为多少
     */
    public void zeroGyro(double reset){
      m_Gyro.setFusedHeading(reset);
    }
    /**
     * 读取车子的朝向
     * @return 车子的朝向，单位为角度
     */
    public double GetYaw() {
      return m_Gyro.getFusedHeading();
    }
    /**
     * 读取车子基于场地的速度
     * @return 速度与角速度
     */
    public FieldRelativeSpeed getFieldRelativeSpeed() {
      return m_FieldRelVel;
    }
    /**
     * 读取车子基于场地的加速度
     * @return 加速度与角速度加速度
     */
    public FieldRelativeAccel getFieldRelativeAccel() {
      return m_FieldRelAccel;
    }

    private void AutoCalibrateOdometry()
    {
      if(RobotContainer.m_Limelight.IsTargetLocked())
      {
          m_IsTargetLocked=true;
          m_TargetLockedTime=Timer.getFPGATimestamp();
      }
      else
      {
          m_IsTargetLocked=false;
      }
      if(m_IsTargetLocked&&Timer.getFPGATimestamp()-m_TargetLockedTime>=Constants.SwerveConstants.AprilTagLockTime)
      {
          RobotContainer.m_SwerveBase.ResetOdometry(RobotContainer.m_Limelight.GetPose2dBotPose());
      }
    }
    
    @Override
    public void periodic() {
      AutoCalibrateOdometry();
      m_FieldRelVel = new FieldRelativeSpeed(getChassisSpeeds(), GetGyroRotation2d());
      m_FieldRelAccel = new FieldRelativeAccel(m_FieldRelVel, m_LastFieldRelVel, GlobalConstants.kLoopTime);
      m_LastFieldRelVel = m_FieldRelVel;
      
      SwerveModuleState[] moduleStates = getStates();
      for(int i=0;i<moduleStates.length;++i)
      {
        moduleStates[i].speedMetersPerSecond*=Constants.SwerveConstants.VelocityCorrectionFactor;
      }
      // This method will be called once per scheduler run
      m_Odometry.update(
        GetGyroRotation2d(), 
        GetPositions());
  
      m_Field.setRobotPose(getPose());
      if(m_EnanbleTelemetry){
        SmartDashboard.putNumber("GetSpeed0", m_SwerveModules[0].GetSpeed());
        SmartDashboard.putNumber("GetSpeed1", m_SwerveModules[1].GetSpeed());
        SmartDashboard.putNumber("GetSpeed2", m_SwerveModules[2].GetSpeed());
        SmartDashboard.putNumber("GetSpeed3", m_SwerveModules[3].GetSpeed());
  
        SmartDashboard.putNumber("Debug/Drive/x meters", getPose().getX());
        SmartDashboard.putNumber("Debug/Drive/y meters", getPose().getY());
        SmartDashboard.putNumber("Debug/Drive/rot radians", getPose().getRotation().getDegrees());
        SmartDashboard.putBoolean("Debug/Drive/isOpenloop", IsOpenLoop);
      }
    }
    private void addShuffleboardDebug(){
      m_SwerveDriveTab.addNumber("GetSpeed0", () ->this.m_SwerveModules[0].GetSpeed())
      .withPosition(0, 0)
      .withSize(1, 1);
      m_SwerveDriveTab.addNumber("GetSpeed1", () ->this.m_SwerveModules[1].GetSpeed())
      .withPosition(0, 1)
      .withSize(1, 1);
      m_SwerveDriveTab.addNumber("GetSpeed2", () ->this.m_SwerveModules[2].GetSpeed())
      .withPosition(0, 2)
      .withSize(1, 1);
      m_SwerveDriveTab.addNumber("GetSpeed3", () ->this.m_SwerveModules[3].GetSpeed())
      .withPosition(0, 3)
      .withSize(1, 1);    
  
      m_SwerveDriveTab.addNumber("x meters", () ->this.getPose().getX())
      .withPosition(1, 0)
      .withSize(1, 1);    
  
      m_SwerveDriveTab.addNumber("y meters", () ->this.getPose().getY())
      .withPosition(1, 1)
      .withSize(1, 1);    
  
      m_SwerveDriveTab.addNumber("rot radians", () ->this.getPose().getRotation().getDegrees())
      .withPosition(1, 2)
      .withSize(1, 1);  
      
      m_SwerveDriveTab.addNumber("FieldRelativeSpeedX", () ->this.getFieldRelativeXVelocity())
      .withPosition(2, 0)
      .withSize(1, 1);
      
      m_SwerveDriveTab.addNumber("FieldRelativeSpeedY", () ->this.getFieldRelativeYVelocity())
      .withPosition(2, 1)
      .withSize(1, 1);
      
      m_SwerveDriveTab.addNumber("FieldRelativeSpeedRot", () ->this.getFieldRelativeAngularVelocity())
      .withPosition(2, 2)
      .withSize(1, 1);
               
    }
    
  
  }
  