// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Start: Import Statements
  // WPILib Required Imports
  import edu.wpi.first.wpilibj.SPI;                              // Serial peripheral interface, used for gyro
  import edu.wpi.first.wpilibj.TimedRobot;                       // Robot Type
  import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;   // Ignore this
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;    // Debug use only on the computer

//START:  Imports for sensors, motors, and inputs - comment what each import is for
  // Kauai Labs
  import com.kauailabs.navx.frc.AHRS;                            // navX-MXP inertial mass unit, has three-axis gyro and accelerometer

  // REV Robotics
  import com.revrobotics.CANSparkMax;                            // Spark MAX controller, CAN port on the roboRIO; controls motors
  import com.revrobotics.CANSparkMaxLowLevel.MotorType;          // Initializes motor types of the Spark MAX motors.

  // WPILib Other Libraries
  import edu.wpi.first.wpilibj.Joystick;                         // Flight stick interface to control the robot's parts
  import edu.wpi.first.wpilibj.drive.DifferentialDrive; // Tank drive - interfacing with the motors of the robot
  import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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
      private DifferentialDrive robot; // Create robot movement object
      private Joystick robotMove_ControlStick; // Create joystick interface object
      private Joystick robotArm_ControlStick; // Create another joystick for controlling the robot arm.
    
    // Variables
    private final double maximum_power = .40; // Base maximum power
    private double DrivePower;  // Power management for robot

    //START: Fixed Numbers - please keep these to a minimum as you cannot edit these in code
      // PORT IDENTIFICATION
        // Motor Control - left/right motor IDs
        private static final int leftMotor_deviceID = 1;
        private static final int rightMotor_deviceID = 2;
        private static final int leftMotor2_deviceID = 3;
        private static final int rightMotor2_deviceID = 4;



    //STOP: Fixed Numbers

    // Motors and Sensors - comment what each does
        private AHRS navX_gyro; // initialize navX gyroscope class to interface with the gyro @ port SPI-MXP

        // Motors
        private MotorControllerGroup robot_leftMotor;
        private CANSparkMax leftMotor_front;  // create object for left motor 
        private CANSparkMax leftMotor_back; 
        private MotorControllerGroup robot_rightMotor;
        private CANSparkMax rightMotor_front; // create object for right motor
        private CANSparkMax rightMotor_back; 

  // STOP: Initialize classes

  /**   
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Link motors, input devices, and sensors to variables.
      leftMotor_front = new CANSparkMax(leftMotor_deviceID, MotorType.kBrushless);
      leftMotor_back = new CANSparkMax(leftMotor2_deviceID, MotorType.kBrushless);
      rightMotor_front = new CANSparkMax(rightMotor_deviceID, MotorType.kBrushless);
      rightMotor_back = new CANSparkMax(rightMotor2_deviceID, MotorType.kBrushless);

    // Connect both motors together to act as one
      robot_leftMotor = new MotorControllerGroup(leftMotor_front, leftMotor_back);
      robot_rightMotor = new MotorControllerGroup(rightMotor_front, rightMotor_back);

      navX_gyro = new AHRS(SPI.Port.kMXP);

    // Joysticks
      robotMove_ControlStick = new Joystick(0);         // joystick that controls the robot
      robotArm_ControlStick = new Joystick(1);      // joystick that controls the robot arm

    // Initialize motors and sensors to neutral point to ensure no misreadings occur.
      navX_gyro.calibrate();

    // Initialize robot
      robot_leftMotor.setInverted(true);
      robot = new DifferentialDrive(robot_leftMotor, robot_rightMotor);
      
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
        // https://pdocs.kauailabs.com/navx-mxp/guidance/yaw-drift/
        // https://pdocs.kauailabs.com/navx-mxp/advanced/techical-references/
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
  }

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
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /*
    Adjust the Drive Power using the slider on the flight stick. Note that the axes of joysticks are always doubles.
    */
    DrivePower = maximum_power * (Math.abs((robotMove_ControlStick.getRawAxis(3) - 1)) / 2); 
    drive(robotMove_ControlStick.getY(), robotMove_ControlStick.getZ(), DrivePower);
    // System.out.println(robotMove_ControlStick.getRawButton(1));
    // System.out.println(navX_gyro.getRoll());
    // robotControl.tankDrive(robot_ControlStick.getY()*DrivePower, robot_ControlStick.getY()*DrivePower);
    // System.out.println(navX_gyro.getPitch());


 
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }

  public void drive(double forward, double turn, double power) {
    robot.arcadeDrive(forward*power, turn*power);   // Wilbert, Ryan
  }

  public void autobalance_robot() {
    float max_incline = 15;
    double autobalance_threshold = 2.5;
    double additive_power;
    double tiltAxis = navX_gyro.getRoll();
    double max_additive_power = 0.1;

    if (tiltAxis > autobalance_threshold) {
      additive_power = (max_additive_power * ((tiltAxis - autobalance_threshold)/(max_incline - autobalance_threshold)));
      if (additive_power < 0) {
        additive_power = 0;
      };
    } else if (tiltAxis > -autobalance_threshold) {
      additive_power = -(max_additive_power * ((tiltAxis + autobalance_threshold)/(-max_incline + autobalance_threshold))); 
      if (additive_power > 0) {
        additive_power = 0;
      };   
    };
  };
  
}
