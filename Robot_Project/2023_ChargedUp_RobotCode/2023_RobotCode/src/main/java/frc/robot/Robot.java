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
  import edu.wpi.first.wpilibj.event.BooleanEvent;
  import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
  import edu.wpi.first.networktables.DoublePublisher;
  import edu.wpi.first.networktables.NetworkTable;
  import edu.wpi.first.networktables.NetworkTableInstance; 
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
  
    // Fixed Numbers - please keep these to a minimum as you cannot edit these in code
      // Motor Control - left/right motor IDs
      private static final int motorID_LF = 1;       // Front Left Motor ID
      private static final int motorID_RF = 2;       // Front Right Motor ID
      private static final int motorID_LR = 3;       // Rear Left Motor ID
      private static final int motorID_RR = 4;       // Rear Right Motor ID
    
    // Motors and Sensors - comment what each does
      private AHRS navX_gyro = new AHRS(SPI.Port.kMXP); // initialize navX gyroscope class to interface with the gyro @ port SPI-MXP

    // Motors - initialize individual motors to later group together
      private CANSparkMax robot_motorLF = new CANSparkMax(motorID_LF, MotorType.kBrushless);  // create object for front left motor
      private CANSparkMax robot_motorLR = new CANSparkMax(motorID_LR, MotorType.kBrushless);  // create object for rear left motor
      private CANSparkMax robot_motorRF = new CANSparkMax(motorID_RF, MotorType.kBrushless);  // create object for front right motor
      private CANSparkMax robot_motorRR = new CANSparkMax(motorID_RR, MotorType.kBrushless);  // create object for rear right motor

    // Connect both motors together to act as one
      private MotorControllerGroup trackL = new MotorControllerGroup(robot_motorLF, robot_motorLR); // create object for left track 
      private MotorControllerGroup trackR = new MotorControllerGroup(robot_motorRF, robot_motorRR); // create object for right track

    // initialize robot and control system
      private DifferentialDrive robot = new DifferentialDrive(trackL, trackR); // Create robot movement object
      private Joystick robot_joystick = new Joystick(0); // Create joystick interface object
    
    // Variables
      private double maximum_power = .40; // Base maximum power
      private double DrivePower;  // Power management for robot

      public NetworkTableInstance inst = NetworkTableInstance.getDefault();
      public NetworkTable autobalance_table = inst.getTable("datatable");
      public DoublePublisher autobalance_cast;
      public double additive_power;

  // STOP: Initialize classes

  // To make it easier for coding team, I listed most of the controls we will be using here.
    double joystickY = robot_joystick.getY(); 
    double joystickX = robot_joystick.getX();
    double joystickZ = robot_joystick.getZ();
    double joystickSlider = robot_joystick.getRawAxis(3);
    BooleanEvent joystickTrigger = robot_joystick.button(1, null);
    BooleanEvent DPad_Up = robot_joystick.povUp(null);
    BooleanEvent DPad_Down = robot_joystick.povDown(null);

    double gyroscope_roll = navX_gyro.getRoll();

    
    

  /**   
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Initialize motors and sensors to ensure no misreadings occur.
      navX_gyro.calibrate();
      trackL.setInverted(true);

    // Initialize robot
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
  public void disabledInit() {
    autobalance_cast= autobalance_table.getDoubleTopic("additive_power").publish();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    autobalance_robot();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() { 
    DrivePower = maximum_power * (Math.abs(joystickSlider - 1)) / 2;  /*  What does this do?
     *    This allows the slider to control the speed of the robot.
     *  Why do we find the absolute value of the slider minus 1 and then divide it by 2?
     *    This normalizes the slider to a range of 0 to 1, as flippig it up makes it go to the negatives. */

    drive(joystickY, joystickX, DrivePower);

 
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    // double tiltAxis = robot_joystick.getY() * 15;      // DEBUG: Emulate gyroscope using joystick
  }

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
    double tiltAxis = gyroscope_roll;
  
    double max_additive_power = 0.1;

    if (tiltAxis > autobalance_threshold) {
      additive_power = (max_additive_power * ((tiltAxis - autobalance_threshold)/(max_incline - autobalance_threshold)));

    } else if (tiltAxis < -autobalance_threshold) {
      additive_power = -(max_additive_power * ((tiltAxis + autobalance_threshold)/(-max_incline + autobalance_threshold))); 
    } else {
      additive_power = 0;
    };

    autobalance_cast.set(additive_power);
  }; 
}
