// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ManualSwerveControll;
import frc.robot.commands.SemiAutoSwerveControll;
import frc.robot.commands.ClimbChargeStation.KeepBalance;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DownPrevention;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private int LastPov=-1;
  private int m_SelectedGrid=0;
  private int m_SelectedArmState=3;
  private int m_SelectedAutoProgram=1;
  private RobotContainer m_RobotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_RobotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    Arm.m_ArmEnabled=false;
  }

  @Override
  public void disabledPeriodic() {
    if(RobotContainer.m_testController.getAButton())
    {
      m_SelectedAutoProgram+=1;
      m_SelectedAutoProgram%=Constants.AutoConstants.AutoBlueCommand.length;
    }
    SmartDashboard.putNumber("SelectedAutoProgram", m_SelectedAutoProgram);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    Arm.m_ArmEnabled=true;
    RobotContainer.m_Arm.SpinTo(0);
    if(DriverStation.getAlliance()==Alliance.Red)
    Constants.AutoConstants.AutoRedCommand[m_SelectedAutoProgram].schedule();
    
    if(DriverStation.getAlliance()==Alliance.Blue)
    Constants.AutoConstants.AutoBlueCommand[m_SelectedAutoProgram].schedule();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // RobotContainer.m_SwerveBase.m_Odometry.resetPosition(new Rotation2d(), RobotContainer.m_SwerveBase.GetPositions(), new Pose2d());
    // RobotContainer.m_SwerveBase.ZeroHeading();
    RobotContainer.m_SwerveBase.WhetherStoreYaw = false;
    Arm.m_ArmEnabled=true;
    RobotContainer.m_Arm.SpinTo(12);
    // CommandScheduler.getInstance().schedule(new CenterControl());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  RobotContainer.m_Arm.offset=(RobotContainer.m_driverController.getRightTriggerAxis()-RobotContainer.m_driverController.getLeftTriggerAxis())*0.1;
  if(RobotContainer.m_driverController.getXButtonPressed())
    ManualSwerveControll.SwitchCarlock();
if(RobotContainer.m_driverController.getXButtonReleased())
    ManualSwerveControll.SwitchCarlock();
if(RobotContainer.m_driverController.getYButtonPressed())
{
  m_SelectedArmState=3;
}
if(RobotContainer.m_driverController.getRightStickButtonPressed())
{
  SemiAutoSwerveControll.GenerateSemiAutoCommand(RobotContainer.m_SwerveBase.getPose(), true, m_SelectedGrid).schedule();;   
}
if(RobotContainer.m_driverController.getLeftStickButtonPressed()&&!RobotContainer.m_driverController.getRightStickButton())
{
  SemiAutoSwerveControll.GenerateSemiAutoCommand(RobotContainer.m_SwerveBase.getPose(), false, m_SelectedGrid).schedule();;   
}
// if(ControllerConstants.SemiAutoBackButton.getAsBoolean())
// {
//     if(!SemiAutoSwerveControll.IsSemiAuto())
//         SemiAutoSwerveControll.GenerateSemiAutoCommand(RobotContainer.m_SwerveBase.m_Odometry.getPoseMeters(),true,m_SelectedGrid).schedule();
    
// }
// if(ControllerConstants.SemiAutoGoButton.getAsBoolean())
// {
//     if(!SemiAutoSwerveControll.IsSemiAuto())
//         SemiAutoSwerveControll.GenerateSemiAutoCommand(RobotContainer.m_SwerveBase.m_Odometry.getPoseMeters(),false,m_SelectedGrid).schedule();
    
// }

if(RobotContainer.m_driverController.getPOV()==0&&LastPov!=0)
{
    m_SelectedArmState-=1;
    m_SelectedArmState=(m_SelectedArmState+7)%7;
}
if(RobotContainer.m_driverController.getPOV()==180&&LastPov!=180)
{
    m_SelectedArmState+=1;
    m_SelectedArmState=(m_SelectedArmState+7)%7;
}
if(RobotContainer.m_driverController.getPOV()==90&&LastPov!=90)
{
    m_SelectedGrid+=1;
    m_SelectedGrid=(m_SelectedGrid+9)%9;
}
if(RobotContainer.m_driverController.getPOV()==270&&LastPov!=270)
{
    m_SelectedGrid-=1;
    m_SelectedGrid=(m_SelectedGrid+9)%9;
}
if(RobotContainer.m_driverController.getLeftBumperPressed())
{
  RobotContainer.m_Arm.SwitchSolenoidStatus();
}
if(RobotContainer.m_driverController.getRightBumperPressed())
{
    RobotContainer.m_Claw.Switch();
}
if(RobotContainer.m_testController.getAButtonPressed())
{
  RobotContainer.m_DownPrevention.Switch();
}
if(RobotContainer.m_driverController.getAButtonPressed())
{
  if(m_SelectedArmState==0)
    RobotContainer.m_Arm.SetState(-5, true);
  if(m_SelectedArmState==1)  
      RobotContainer.m_Arm.SetState(30, true);
      if(m_SelectedArmState==2)
      RobotContainer.m_Arm.SetState(70, true);
      if(m_SelectedArmState==3)
      RobotContainer.m_Arm.SetState(12, true);
      if(m_SelectedArmState==4)
      { 
        
      RobotContainer.m_Arm.SetState(100, false);
      
      }if(m_SelectedArmState==5)
      {
        RobotContainer.m_Arm.SetState(140, true);
      }
 
      if(m_SelectedArmState==6)
      RobotContainer.m_Arm.SetState(-40, true);
         
 
}
if(m_SelectedArmState==0)
SmartDashboard.putString("SelectedArmState", "GetDown??");
if(m_SelectedArmState==1)
    SmartDashboard.putString("SelectedArmState", "Get!!!!!");
    if(m_SelectedArmState==2)
  SmartDashboard.putString("SelectedArmState", "PPTGet?????");
  
  if(m_SelectedArmState==3)
  SmartDashboard.putString("SelectedArmState", "Original.");
  
  if(m_SelectedArmState==4)
  SmartDashboard.putString("SelectedArmState", "BackFangClose");
  
  if(m_SelectedArmState==5)
  SmartDashboard.putString("SelectedArmState", "BackFangFar");

  if(m_SelectedArmState==6)
  SmartDashboard.putString("SelectedArmState", "GetDown!!!");


SmartDashboard.putNumber("ArmState", m_SelectedArmState);

// RobotContainer.m_Arm.SpinAt((RobotContainer.m_driverController.getRightTriggerAxis()-RobotContainer.m_driverController.getLeftTriggerAxis())*0.5);
LastPov=RobotContainer.m_driverController.getPOV();   SmartDashboard.putNumber("SelectedGrid", m_SelectedGrid);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
