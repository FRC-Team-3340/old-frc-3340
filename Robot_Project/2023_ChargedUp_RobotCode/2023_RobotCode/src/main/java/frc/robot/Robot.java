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

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  // Autonomous
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private static final String kAutobalance = "Autobalance";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();  

  // CANSparkMax IDs
    private static final int motorID_LF = 1;       // Front Left Motor ID
    private static final int motorID_RF = 2;       // Front Right Motor ID
    private static final int motorID_LR = 3;       // Rear Left Motor ID
    private static final int motorID_RR = 4;       // Rear Right Motor ID
    private static final int motorID_gripper = 7;  // Arm Motor ID
    private static final int motorID_arm = 9;
  
  private MotorType motor_type = MotorType.kBrushless;    // Type of motor is brushless. Do not change.

  // Motor controllers
    private CANSparkMax motorL_front = new CANSparkMax(motorID_LF, motor_type); 
    private CANSparkMax motorL_rear = new CANSparkMax(motorID_LR, motor_type); 
    private CANSparkMax motorR_front = new CANSparkMax(motorID_RF, motor_type);  
    private CANSparkMax motorR_rear = new CANSparkMax(motorID_RR, motor_type); 
    private CANSparkMax motor_arm = new CANSparkMax(motorID_arm, motor_type);
    private CANSparkMax motor_gripper = new CANSparkMax(motorID_gripper, motor_type); 


  // Motor, sensor, and input config
    private AHRS navX_gyro = new AHRS(SPI.Port.kMXP); // navX gyroscope object, SPI-MXP
    private Joystick robot_joystick = new Joystick(0); // Create joystick interface object
    private Joystick arm_joystick = new Joystick(1);
    public RelativeEncoder arm_encoder = motor_arm.getEncoder();

    private double MaxPower = 1; // Base maximum power
    
  // Create objects for both motor pairs to act as one
    private MotorControllerGroup left_tread = new MotorControllerGroup(motorL_front, motorL_rear);
    private MotorControllerGroup right_tread = new MotorControllerGroup(motorR_front, motorR_rear);

  // initialize robot and control system
    private DifferentialDrive robot = new DifferentialDrive(left_tread, right_tread); 
 
  // Logging and debugging utilities
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable stats_table = inst.getTable("datatable");
    public DoublePublisher ab_publisher;
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
    */

      public SparkMaxLimitSwitch arm_forwardLimit;
      public SparkMaxLimitSwitch arm_reverseLimit;

  /**   
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {
    // Initialize motors and sensors to ensure no misreadings occur.
      navX_gyro.calibrate();
      left_tread.setInverted(true);

    // Initialize robot
      m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
      m_chooser.addOption("My Auto", kCustomAuto);
      SmartDashboard.putData("Auto choices", m_chooser);

    // Initialize robot arm and gripper
      motor_arm.restoreFactoryDefaults();
      motor_gripper.restoreFactoryDefaults();

      motor_arm.setSoftLimit(SoftLimitDirection.kForward, 1);
      motor_arm.setSoftLimit(SoftLimitDirection.kReverse, -36);

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
          move_robot(autobalance_power, 0, 1, false);

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
   move_robot(robot_joystick.getY(), robot_joystick.getX(), robot_joystick.getRawAxis(3), true);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    ab_publisher = stats_table.getDoubleTopic("Autobalance Power").publish();
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
    move_robot(robot_joystick.getY(), robot_joystick.getX(), robot_joystick.getRawAxis(3), true);
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

  public void move_robot(double forward, double turn, double speed, boolean isUserInput) {
    if (isUserInput == true) {
      speed = MaxPower * (Math.abs(speed - 1)) / 2;   
    }
    robot.arcadeDrive(forward*speed, turn*speed);   // Wilbert, Ryan
  }

  public void autobalance_robot(double source) {
    float maxAngle = 15;
    double minAngle = 2.5;
    double abAxis = source;
 
    double max_additive_power = 0.2;

    if (abAxis > maxAngle)  {
        autobalance_power = -max_additive_power;
    } else if (abAxis < -maxAngle) {
      autobalance_power = max_additive_power;

    } else if (abAxis > minAngle) {
        autobalance_power = -(max_additive_power * ((abAxis - minAngle)/(maxAngle - minAngle)));
      
    } else if (abAxis < -minAngle) {
      autobalance_power = (max_additive_power * ((abAxis + minAngle)/(-maxAngle + minAngle))); 
  
    } else {
      autobalance_power = 0;
    };
      
      // gyroRoll_output.set(gyroscope_roll);
      ab_publisher.set(autobalance_power);
  }; 

  public void move_robot_arm(double encoder, double input, boolean force) {
    if (force == true) {
      // Goal: rotate to a specific point


    }
    else {
      motor_arm.set(input);
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