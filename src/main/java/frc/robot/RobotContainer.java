// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CenterControl;
import frc.robot.commands.ManualSwerveControll;
import frc.robot.commands.SemiAutoSwerveControll;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pixy;
import frc.robot.subsystems.SwerveBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DownPrevention;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  
  private static RobotContainer instance ;
  public static SwerveBase m_SwerveBase=SwerveBase.getInstance();
  public static Limelight m_Limelight=Limelight.getInstance();
  public static XboxController m_driverController = new XboxController(0);
  public static XboxController m_testController = new XboxController(1);
  public static Arm m_Arm=Arm.GetInstance();
  public static Pixy m_Pixy2=Pixy.getInstance();
  public static Claw m_Claw=Claw.GetInstance();
  public static DownPrevention m_DownPrevention=DownPrevention.GetInstance();
  // The robot's subsystems and commands are defined here...
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    m_SwerveBase.setDefaultCommand(new ManualSwerveControll());
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() 
  {
    // new JoystickButton(m_driverController, 1).onTrue(new InstantCommand(() -> m_Arm.SpinTo(45)));
    
    // new JoystickButton(m_driverController, 1).onFalse(new InstantCommand(() -> m_Arm.SpinTo(0)));
    // new JoystickButton(m_driverController, 2).onTrue(new InstantCommand(() -> m_Arm.SwitchSolenoidStatus()));
    // // new JoystickButton(m_driverController, 2).onTrue(new InstantCommand(() -> m_Arm.SpinPositive()));
  
    // // new JoystickButton(m_driverController, 2).onFalse(new InstantCommand(() -> m_Arm.Stop()));
    // new JoystickButton(m_driverController,3).onTrue(new InstantCommand(()->m_Claw.Close()));
    // new JoystickButton(m_driverController,3).onFalse(new InstantCommand(()->m_Claw.Ease()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static RobotContainer getInstance() {
    if (instance == null){
        instance = new RobotContainer();
    }
    return instance;
  }

}
