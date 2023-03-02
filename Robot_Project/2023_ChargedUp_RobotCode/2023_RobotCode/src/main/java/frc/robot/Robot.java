// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPILib Imports
    import edu.wpi.first.wpilibj.SPI;                              // Serial peripheral interface, used 
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
    import com.kauailabs.navx.frc.AHRS;                            // navX-MXP inertial mass unit, has three-axis gyro and accelerometer
    import com.revrobotics.CANSparkMax;                            // Spark MAX controller, CAN port on the roboRIO; controls motors
    import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
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
      private static final int motorID_gripper = 7;      // Arm Motor ID
      private static final int motorID_arm = 9;
    
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
      private Joystick arm_joystick = new Joystick(1); 
    
    // Variables
      private double maximum_power = 1; // Base maximum power
      private double DrivePower;  // Power management for robot

      // DEBUGGING TOOLS
      public NetworkTableInstance inst = NetworkTableInstance.getDefault();
      public NetworkTable stats_table = inst.getTable("datatable");
      public DoublePublisher NT_Cast;
      public DoublePublisher robot_forwardSpeed;
      public DoublePublisher robot_rotationSpeed;
      public DoublePublisher gyroRoll_output;
      public DoublePublisher ArmEncoderOutput;
      public double autobalance_power;

      /* Important commands for getting user input:
       * joystick.getX(), .getY(), .getZ()
       * --> get input from the joystick: tilting forward/back, side to side, and twisting respectively.
       * --> returns a double
       * 
       * joystick.getRawAxis(axis)
       * --> get input from another axis on the joystick. Use axis 3 for the slider on the flight stick.
       * --> returns a double
       * 
       * joystick.getRawButton(button)
       * --> gets the state (pressed/unpressed) of a button on the joystick (1-16). 
       * --> returns a boolean, can be converted to integer by writing this code, replacing the pseudocode:
       *     *this_button* = *joystick*.getRawButton(*button*) 
       * 
       * gyro.getYaw(), .getPitch(), .getRoll()
       * --> gets the yaw, pitch, or roll input of the gyroscope (tilt).
       * --> returns a double
       * 
       * 
       * 
       * 
       */

      public RelativeEncoder arm_encoder = robot_motorArm.getEncoder();

      public SparkMaxLimitSwitch arm_forwardLimit;
      public SparkMaxLimitSwitch arm_reverseLimit;

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
      robot_motorArm.setSoftLimit(SoftLimitDirection.kReverse, -36);

      // arm_forwardLimit = robot_motorArm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
      // arm_reverseLimit = robot_motorArm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);


    }

    

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * SmartDashboard integrated updating.
   */

  @Override
  public void robotPeriodic() {
  ArmEncoderOutput.set(arm_encoder.getPosition());
  
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
          double autobalance_axis = navX_gyro.getRoll();

          autobalance_robot(autobalance_axis);
          move_robot(autobalance_power, 0, 1);

          if (autobalance_power == 0) {
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
    DrivePower = maximum_power * (Math.abs(robot_joystick.getRawAxis(3) - 1)) / 2;  /*  What does this do?
    *    This allows the slider to control the speed of the robot.
    *  Why do we find the absolute value of the slider minus 1 and then divide it by 2?
    *    This normalizes the slider to a range of 0 to 1, as flippig it up makes it go to the negatives. */
   move_robot(robot_joystick.getY(), robot_joystick.getX(), DrivePower);

        // insert code that you want the robot to process periodically during teleop.
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    NT_Cast = stats_table.getDoubleTopic("Autobalance Power").publish();
    robot_forwardSpeed = stats_table.getDoubleTopic("Robot Foward Power").publish();
    robot_rotationSpeed = stats_table.getDoubleTopic("Robot Rotation Power").publish();
    ArmEncoderOutput = stats_table.getDoubleTopic("Arm Motor Rotations").publish();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    arm_encoder.setPosition(0);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {  
    move_robot_arm(arm_encoder.getPosition(), arm_joystick.getY(), false);
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

  public void move_robot(double forward, double turn, double power) {
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
        autobalance_power = - max_additive_power;

      } else if (tiltAxis < -max_incline) {
            autobalance_power = max_additive_power;

      } else if (tiltAxis > autobalance_threshold) {
          autobalance_power = -(max_additive_power * ((tiltAxis - autobalance_threshold)/(max_incline - autobalance_threshold)));
        
      } else if (tiltAxis < -autobalance_threshold) {
        autobalance_power = (max_additive_power * ((tiltAxis + autobalance_threshold)/(-max_incline + autobalance_threshold))); 
    
      } else {
        autobalance_power = 0;
      };
      
      // gyroRoll_output.set(gyroscope_roll);
      NT_Cast.set(autobalance_power);
  }; 

  public void move_robot_arm(double encoder, double input, boolean force) {
    if (force == true) {
      // Goal: rotate to a specific point


    }
    else {
      robot_motorArm.set(input);
    }
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
}