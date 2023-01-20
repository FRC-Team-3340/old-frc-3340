// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.SPI; // Serial peripheral interface, used for gyro
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Imports for sensors, motors, and inputs - comment what each import is for
import com.kauailabs.navx.frc.AHRS; // navX-MXP inertial mass unit, has three-axis gyro and accelerometer
import edu.wpi.first.wpilibj.Joystick; // Flight stick interface to control the robots
import edu.wpi.first.wpilibj.drive.DifferentialDrive; // Tank drive - interfacing with the motors of the robot


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  /* 
  What's the difference between public/private in Java?
    > Declaring something as public or private changes the scope of the particular variable, function, or class.
        * private -> Scope is limited to the particular class or function only.
        * public -> Scope is global and can be accessed by other functions, classes, or scripts.
   */
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Sensors, Motors, and Inputs - Comment your variables
  private AHRS navX_gyro = new AHRS(SPI.Port.kMXP); // Create a variable that interfaces with the navX gyro @ port SPI-MXP

  /**   
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    /*
      Upon initialization of the robot, the gyroscope must always be calibrated.  
      This is done to ensure the gyroscope is reading a default position upon runtime.
      Failure to calibrate the gyroscope will result in misreadings that may affect the robot.
    */
    navX_gyro.calibrate(); // Calibrate the gyro so that it is set to 0 on all axes. 

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    navX_gyro.getAngle(); // Get the rotation of the gyro every 20ms. Debug only
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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // insert code that you want the robot to process periodically during teleop.
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
  public void simulationPeriodic() {
    navX_gyro.getAngle(); // Get the rotation of the gyro every 20ms. Debug only
  }
}
