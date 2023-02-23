// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPILib Imports
  // Required imports
    import edu.wpi.first.wpilibj.SPI;                              // Serial peripheral interface, used for gyro
    import edu.wpi.first.wpilibj.TimedRobot;                       // Robot Type
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;   // Ignore this
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;    // Debug use only on the computer

    // WPILib Object Libraries and Inputs
    import edu.wpi.first.wpilibj.drive.DifferentialDrive;
    import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
    import edu.wpi.first.wpilibj.Joystick;                         // Flight stick interface to control the robot's parts
    import edu.wpi.first.wpilibj.event.BooleanEvent;

  // Network Table
    import edu.wpi.first.networktables.NetworkTable;
    import edu.wpi.first.networktables.NetworkTableInstance; 
    import edu.wpi.first.networktables.DoublePublisher;


// Imports for sensors, motors, and inputs - comment what each import is for
  // Kauai Labs
  import com.kauailabs.navx.frc.AHRS;                            // navX-MXP inertial mass unit, has three-axis gyro and accelerometer
  // REV Robotics
  import com.revrobotics.CANSparkMax;                            // Spark MAX controller, CAN port on the roboRIO; controls motors
  import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;          // Initializes motor types of the Spark MAX motors.
  import com.revrobotics.REVLibError;

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
    // Autonomous
      private static final String kDefaultAuto = "Default";
      private static final String kCustomAuto = "My Auto";
      private static final String kAutobalance = "Autobalance";
      private String m_autoSelected;
      private final SendableChooser<String> m_chooser = new SendableChooser<>();  
  
    // Fixed Numbers - please keep these to a minimum as you cannot edit these in code
      // Motor Control - left/right motor IDs
      private static final int motorID_LF = 1;       // Front Left Motor ID
      private static final int motorID_RF = 2;       // Front Right Motor ID
      private static final int motorID_LR = 3;       // Rear Left Motor ID
      private static final int motorID_RR = 4;       // Rear Right Motor ID
      private static final int motorID_arm = 7;      // Arm Motor ID
      private static final int motorID_gripper = 9;
    
    // Motors and Sensors - comment what each does
      private AHRS navX_gyro = new AHRS(SPI.Port.kMXP); // initialize navX gyroscope class to interface with the gyro @ port SPI-MXP

    // Motors - initialize individual motors to later group together
      private CANSparkMax robot_motorLF = new CANSparkMax(motorID_LF, MotorType.kBrushless);  // create object for front left motor
      private CANSparkMax robot_motorLR = new CANSparkMax(motorID_LR, MotorType.kBrushless);  // create object for rear left motor
      private CANSparkMax robot_motorRF = new CANSparkMax(motorID_RF, MotorType.kBrushless);  // create object for front right motor
      private CANSparkMax robot_motorRR = new CANSparkMax(motorID_RR, MotorType.kBrushless);  // create object for rear right motor
      private CANSparkMax robot_motorArm = new CANSparkMax(motorID_arm, MotorType.kBrushless);
      private CANSparkMax robot_motorGripper = new CANSparkMax(motorID_gripper, MotorType.kBrushless); 
      
    // Connect both motors together to act as one
      private final MotorControllerGroup trackL = new MotorControllerGroup(robot_motorLF, robot_motorLR); // create object for left track 
      private final MotorControllerGroup trackR = new MotorControllerGroup(robot_motorRF, robot_motorRR); // create object for right track
      // The order in which you establish a final variable does not matter. 

    // initialize robot and control system
      private final DifferentialDrive robot = new DifferentialDrive(trackL, trackR); // Create robot movement object
      private Joystick robot_joystick = new Joystick(0); // Create joystick interface object
    
    // Variables
      private double maximum_power = 1; // Base maximum power
      private double DrivePower;  // Power management for robot

      // DEBUGGING TOOLS
      public NetworkTableInstance inst = NetworkTableInstance.getDefault();
      public NetworkTable stats_table = inst.getTable("datatable");
      public DoublePublisher autobalance_cast;
      public DoublePublisher robot_forwardSpeed;
      public DoublePublisher robot_rotationSpeed;
      public DoublePublisher gyroRoll_output;
      public double additive_power;

      private double gyroscope_roll;

      private double joystickX;
      private double joystickY;
      private double joystickZ; // This gets the input from twisting the stick.
      private double joystickSlider;

      private BooleanEvent joystickTrigger;

      public RelativeEncoder arm_encoder = robot_motorArm.getEncoder();

  // STOP: Initialize classes4



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

    // Initialize robot arm and gripper
      robot_motorArm.restoreFactoryDefaults();
      robot_motorGripper.restoreFactoryDefaults();

      robot_motorArm.setSoftLimit(SoftLimitDirection.kForward, 1);
      robot_motorArm.setSoftLimit(SoftLimitDirection.kReverse, -1);
      robot_motorArm.enableSoftLimit(SoftLimitDirection.kForward, true);
      robot_motorArm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
    update_sensor_data();
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
      case kAutobalance: 
        boolean balanced = false;
        float time_balanced = 0;
        while (balanced == false) {
          autobalance_robot(gyroscope_roll);
          drive(additive_power, 0, 1);
          if (additive_power == 0) {
            time_balanced++;
          }
          if (time_balanced == 10000) {
            System.out.println("Robot is balanced :)");
            balanced = true;
          }
        }
        if (balanced == true) {
          break;
        }
      case kDefaultAuto:
      default:
        
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
    robot_motorArm.set(joystickY);
        // insert code that you want the robot to process periodically during teleop.
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    autobalance_cast = stats_table.getDoubleTopic("Autobalance Power").publish();
    robot_forwardSpeed = stats_table.getDoubleTopic("Robot Foward Power").publish();
    robot_rotationSpeed = stats_table.getDoubleTopic("Robot Rotation Power").publish();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
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
    robot_forwardSpeed.set(forward*power);
    robot_rotationSpeed.set(turn*power);
  }

  public void autobalance_robot(double input_source) {
    float max_incline = 15;
    double autobalance_threshold = 2.5;
    double tiltAxis = input_source;
 
    double max_additive_power = 0.2;

    if (tiltAxis > max_incline) {
        additive_power = - max_additive_power;

      } else if (tiltAxis < -max_incline) {
            additive_power = max_additive_power;

      } else if (tiltAxis > autobalance_threshold) {
          additive_power = -(max_additive_power * ((tiltAxis - autobalance_threshold)/(max_incline - autobalance_threshold)));
        
      } else if (tiltAxis < -autobalance_threshold) {
        additive_power = (max_additive_power * ((tiltAxis + autobalance_threshold)/(-max_incline + autobalance_threshold))); 
    
      } else {
        additive_power = 0;
      };
      
      // gyroRoll_output.set(gyroscope_roll);
      autobalance_cast.set(additive_power);
  }; 

  public void update_sensor_data() {
    // Update Joystick analog inputs:
    joystickX = robot_joystick.getX();
    joystickY = robot_joystick.getY();
    joystickZ = robot_joystick.getZ(); 
    joystickSlider = robot_joystick.getRawAxis(3);
    
    // Update gyroscope axis:
    gyroscope_roll = navX_gyro.getRoll();

    // Update encoder rotation:
    arm_encoder = robot_motorArm.getEncoder();

  }

  /*
    function move_robot_arm(movement_input, overwrite) {
      get current rotation of arm motor
      if motor is not exceeding 10% or 90% rotation {
        if (overwrite === true) {
          set rotation to movement_input, usually given by a preset button.
          buttons 8, 10, and 12 will trigger this event
        } else {
          add rotation to movement_input, as controlled by the D-Pad
        }
      }
      do this for negative and positive if using the D-Pad.
    }
   */

   /*
    // TODO: Create pseudocode for gripper
    function control_gripper() {

    }
    */
}

