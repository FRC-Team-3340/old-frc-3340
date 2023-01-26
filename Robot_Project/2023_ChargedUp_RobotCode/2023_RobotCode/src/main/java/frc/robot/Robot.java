// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Start: Import Statements
  // WPILib Required Imports
  import edu.wpi.first.wpilibj.SPI; // Serial peripheral interface, used for gyro
  import edu.wpi.first.wpilibj.TimedRobot;  // Robot Type
  import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;  // Ignore this
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Debug use only on the computer

  //START:  Imports for sensors, motors, and inputs - comment what each import is for

  // Kauai Labs
  import com.kauailabs.navx.frc.AHRS; // navX-MXP inertial mass unit, has three-axis gyro and accelerometer;;

  // REV Robotics
  import com.revrobotics.CANSparkMax; // Spark MAX controller through the CAN port on the roboRIO, controls motors;
  import com.revrobotics.CANSparkMaxLowLevel.MotorType;   // Initializes motor types of the Spark MAX motors.

  // WPILib Other Libraries
  import edu.wpi.first.wpilibj.Joystick; // Flight stick interface to control the robots
  import edu.wpi.first.wpilibj.drive.DifferentialDrive; // Tank drive - interfacing with the motors of the robot

  //STOP: Imports for sensors, motors, and inputs
//STOP: Import Statements

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


  // START: initialize classes:

    // WPILib pre-defined variables
      private static final String kDefaultAuto = "Default";
      private static final String kCustomAuto = "My Auto";
      private String m_autoSelected;
      private final SendableChooser<String> m_chooser = new SendableChooser<>();  
  
    // initialize robot and control system
      private DifferentialDrive robot_3340; // Create robot movement object
      private Joystick flightstick; // Create joystick interface object
    
    //START: Fixed values - please keep these to a minimum as you cannot edit these in code
      // PORT IDENTIFICATION
        // Motor Control - left/right motor IDs
        private static final int leftMotor_deviceID = 1;
        private static final int rightMotor_deviceID = 3;
    //STOP: Fixed Numbers

    // Motors and Sensors - comment what each does
        private AHRS navX_gyro; // initialize navX gyroscope class to interface with the gyro @ port SPI-MXP

        // Motors
        private CANSparkMax robot_leftMotor;  // create object for left motor
        private CANSparkMax robot_rightMotor; // create object for right motor
  // STOP: Initialize classes

  /**   
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Link motors, input devices, and sensors to variables.
      robot_leftMotor = new CANSparkMax(leftMotor_deviceID, MotorType.kBrushless);
      robot_rightMotor = new CANSparkMax(rightMotor_deviceID, MotorType.kBrushless);
      navX_gyro = new AHRS(SPI.Port.kMXP);
      flightstick = new Joystick(0);

    // Initialize motors and sensors to neutral point to ensure no misreadings occur.
      robot_leftMotor.restoreFactoryDefaults();
      robot_rightMotor.restoreFactoryDefaults();
      navX_gyro.calibrate();

    // Initialize robot
      robot_3340 = new DifferentialDrive(robot_leftMotor, robot_rightMotor);
      

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
  public void testPeriodic() {
    robot_3340.tankDrive(flightstick.getX(), flightstick.getY());
    System.out.println(navX_gyro.getPitch());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    System.out.println(navX_gyro.getPitch());
    navX_gyro.getAngle(); // Get the rotation of the gyro every 20ms. Debug only
  }
}
