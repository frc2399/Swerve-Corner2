// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.math.util.Units;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public Joystick stick = new Joystick(0);


  public WPI_CANCoder cancoder = new WPI_CANCoder(21);

  public TalonFX steer = new TalonFX(22);

  public TalonFX drive = new TalonFX(23);

  public PIDController ccpid = new PIDController(0.01, 0, 0.0001);

  public JoystickButton homebutton = new JoystickButton(stick, 2);

  public PIDController talpid = new PIDController(0.0001, 0, 0);

  public double talon_mk4i_180_count = 21943;

  public double prev_ang = 0;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("angle", 0);
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double Xj = Math.pow(stick.getRawAxis(4), 3);
    double Yj = -Math.pow(stick.getRawAxis(5), 3);

    SmartDashboard.putNumber("Xj", Xj);
    SmartDashboard.putNumber("Yj", Yj);
    SmartDashboard.putNumber("cancoder", cancoder.getAbsolutePosition());

    double str = Xj;
    double fwd = Yj;
    double vel = MathUtil.clamp(Math.sqrt((Xj * Xj) + (Yj * Yj)), -1, 1);
    double ang = Units.radiansToDegrees(-Math.atan2(str, fwd));
    ang = prev_ang + ((ang - prev_ang) + 180) % 360 - 180;

    SmartDashboard.putNumber("velocity", vel);
    SmartDashboard.putNumber("angle", ang);

    //steer.set(ControlMode.PercentOutput, Yj * 0.5);




 // double targetAngle =  SmartDashboard.getNumber("angle", 0);
  double targetAngle =  231;


    // double co = MathUtil.clamp(
    //   ccpid.calculate(cancoder.getAbsolutePosition(),
    //   targetAngle), -1, 1);

    // steer.set(ControlMode.PercentOutput, -1 * co);


    if (stick.getRawButton(2) == true)
    {
      double co = MathUtil.clamp(
      ccpid.calculate(cancoder.getAbsolutePosition(),
      targetAngle), -1, 1);
      steer.set(ControlMode.PercentOutput, -1 * co);
      steer.setSelectedSensorPosition(0);
      System.out.println("HELLO @_@");
      talpid.reset();
      return;
    }
  

    double sp = -talon_mk4i_180_count * (ang/180);

    double co = MathUtil.clamp(talpid.calculate(steer.getSelectedSensorPosition(), sp), -1, 1);

    steer.set(ControlMode.PercentOutput, co);

    drive.set(ControlMode.PercentOutput, vel);

    prev_ang = ang;
    
  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
