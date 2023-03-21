// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// WPILib Imports

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot; // Robot Type
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // Chooser for autonomous
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Debug use only on the computer

// WPILib Object Libraries and Inputs
import edu.wpi.first.wpilibj.drive.DifferentialDrive;                 // To use arcade drive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;       // Conjoins two motors as one
import edu.wpi.first.wpilibj.DigitalInput;                            // Limit Switch interface for the robot's arm
import edu.wpi.first.wpilibj.Joystick;                                // Joystick interface for controlling the robot
import edu.wpi.first.math.controller.PIDController;

// Network Table
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;

// Imports for sensors, motors, and inputs - comment what each import is for
import com.kauailabs.navx.frc.AHRS; // navX-MXP IMU that has a useful gyroscope
import com.revrobotics.CANSparkMax; // Spark MAX motor controller
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; // Initializes motor types of the Spark MAX motors.

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

 // UNLESS ABSOLUTELY NECCESARY, DO NOT CHANGE ANYTHING DEEMED "FINAL"

public class Robot extends TimedRobot {
    // Autonomous
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private static final String kAutobalance = "Autobalance";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // CANSparkMax IDs -> These you can change in an emergency despite being final.
    private static final int motorID_LF = 1;        // Front Left Motor ID
    private static final int motorID_RF = 2;        // Front Right Motor ID
    private static final int motorID_LR = 3;        // Rear Left Motor ID
    private static final int motorID_RR = 4;        // Rear Right Motor ID
    private static final int motorID_gripper = 7;   // Gripper Motor ID
    private static final int motorID_arm = 9;       // Arm Motor ID

    private final MotorType motor_type = MotorType.kBrushless; // Type of motor is brushless. Do not change.

    // Motor controllers
    private final CANSparkMax motorL_front = new CANSparkMax(motorID_LF, motor_type);
    private final CANSparkMax motorL_rear = new CANSparkMax(motorID_LR, motor_type);
    private final CANSparkMax motorR_front = new CANSparkMax(motorID_RF, motor_type);
    private final CANSparkMax motorR_rear = new CANSparkMax(motorID_RR, motor_type);
    private final CANSparkMax motor_arm = new CANSparkMax(motorID_arm, motor_type);
    private final CANSparkMax motor_gripper = new CANSparkMax(motorID_gripper, motor_type);

    // Motor, sensor, and input config
    private AHRS navX_gyro = new AHRS(SPI.Port.kMXP); // navX gyroscope object, SPI-MXP
    
    // merge both together
    private Joystick robot_joystick = new Joystick(0); // Create joystick interface object
    private Joystick arm_joystick = new Joystick(1);
    private Joystick emulated_gyroscope = new Joystick(2); // ignore this

    public RelativeEncoder arm_encoder = motor_arm.getEncoder();
    public RelativeEncoder gripper_encoder = motor_gripper.getEncoder();


    // Create objects for both motor pairs to act as one, don't tamper
    private final MotorControllerGroup left_tread = new MotorControllerGroup(motorL_front, motorL_rear);
    private final MotorControllerGroup right_tread = new MotorControllerGroup(motorR_front, motorR_rear);

    // initialize robot and control system
    private DifferentialDrive robot = new DifferentialDrive(left_tread, right_tread);
    
    // FINE-TUNE: Controls rotation speed to preset option.
    private PIDController rotate_to = new PIDController(0.01, 0.01, 0.5);

    // LIMIT SWITCHES
    public DigitalInput reverse_switch = new DigitalInput(0);
    public DigitalInput forwards_switch = new DigitalInput(1);

    // GLOBAL FOR SMART DASHBOARD.
    private double max_drivePower = 0.5; // Base maximum power for driving the robot
    private double max_armPower = 0.05;
    private boolean limitSwitch_override = false; // IF LIMIT SWITCH BREAKS, SET TO TRUE ON SMARTDASHBOARD OR HERE.

    // Logging and debugging utilities
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable stats_table = inst.getTable("datatable");
    public DoublePublisher ab_publisher;
    public DoublePublisher gyroRoll_output;
    public DoublePublisher ArmEncoderOutput;
    public DoublePublisher GripperOutput;
    public double autobalance_power;
    public int lastPressed = 0;

    /*
     * Important commands for getting user input:
     * joystick.getX(), .getY(), .getZ()
     * --> get input from the joystick: tilting forward/back, side to side, and
     * twisting respectively.
     * --> returns a double
     *
     * joystick.getRawAxis(axis)
     * --> get input from another axis on the joystick. Use axis 3 for the slider on
     * the flight stick.
     * --> returns a double
     *
     * joystick.getRawButton(button)
     * --> gets the state (pressed/unpressed) of a button on the joystick (1-16).
     * --> returns a boolean, can be converted to integer by writing this code,
     * replacing the pseudocode:
     * *this_button* = *joystick*.getRawButton(*button*)
     *
     * gyro.getYaw(), .getPitch(), .getRoll()
     * --> gets the yaw, pitch, or roll input of the gyroscope (tilt).
     * --> returns a double
     */

    /**
     * This function is run when the robot is first started up and should be used
     * for any
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
        m_chooser.addOption("Autobalance", kAutobalance);
        SmartDashboard.putData("Auto choices", m_chooser);

        SmartDashboard.putNumber("Arm Position (Encoder)", arm_encoder.getPosition());
        SmartDashboard.putNumber("Gripper Position (Encoder)", gripper_encoder.getPosition());
        SmartDashboard.putNumber("Maximum Drive Power", max_drivePower);
        SmartDashboard.putNumber("Maximum Arm Power", max_armPower);
        SmartDashboard.putBoolean("Disable Limit Switches", limitSwitch_override);

        // Initialize robot arm and gripper
        motor_arm.restoreFactoryDefaults();
        motor_gripper.restoreFactoryDefaults();

        motor_gripper.setIdleMode(IdleMode.kBrake);
        motor_gripper.setSoftLimit(SoftLimitDirection.kReverse, -40);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     * <p>
     * SmartDashboard integrated updating.
     */

    @Override
    public void robotPeriodic() {

    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */

    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /**
     * This function is called periodically during autonomous.
     */
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
                    autobalance_power = autobalance_robot(navX_gyro.getRoll());
                    move_robot(autobalance_power, 0, 1, false);
                    if (autobalance_power == 0) {
                        time_balanced++;
                    }
                    if (time_balanced == 10000) {
                        System.out.println("Robot is balanced :)");
                        balanced = true;
                        break;
                    };
                }; //Why these alone? =(
            case kDefaultAuto:
            default:

                break;
        }
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        gripper_encoder.setPosition(0);
        motor_gripper.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        limitSwitch_override = SmartDashboard.getBoolean("Forward Limit Enabled", false);

        move_robot(robot_joystick.getY(), robot_joystick.getX(), robot_joystick.getRawAxis(3), true);
        
        if (Math.abs(arm_joystick.getY()) > 0.1) {
            move_robot_arm(false, arm_joystick.getY(), 0);
        } else if (arm_joystick.getRawButton(8) == true) {
            move_robot_arm(true, 0, -5);
        } else if (arm_joystick.getRawButton(10) == true) {
            move_robot_arm(true, 0, -20);
        } else if (arm_joystick.getRawButton(12) == true) {
            move_robot_arm(true, 0, -30);
        } else {
            move_robot_arm(false, 0, 0);
        }

        autobalance_robot(emulated_gyroscope.getY());

        // toggle_gripper(arm_joystick.getRawButton(1));
    
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        motor_gripper.enableSoftLimit(SoftLimitDirection.kReverse, false);
        ab_publisher = stats_table.getDoubleTopic("Autobalance Power").publish();
        ArmEncoderOutput = stats_table.getDoubleTopic("Arm Motor Rotations").publish();
        GripperOutput = stats_table.getDoubleTopic("Gripper Encoder").publish();
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
        motor_gripper.set(0.05);
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        // arm_encoder.setPosition(0);
    }

    /* This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        limitSwitch_override = SmartDashboard.getBoolean("Forward Limit Enabled", false);

        move_robot(robot_joystick.getY(), robot_joystick.getX(), robot_joystick.getRawAxis(3), true);

        if (Math.abs(arm_joystick.getY()) > 0.1) {
            move_robot_arm(false, arm_joystick.getY(), 0);
        } else if (arm_joystick.getRawButton(8) == true) {
            move_robot_arm(true, 0, -5);
        } else if (arm_joystick.getRawButton(10) == true) {
            move_robot_arm(true, 0, -20);
        } else if (arm_joystick.getRawButton(12) == true) {
            move_robot_arm(true, 0, -30);
        } else {
            move_robot_arm(false, 0, 0);
        }

        if (Math.abs(emulated_gyroscope.getY()) < .1) {
            autobalance_robot(navX_gyro.getRoll());
        } else {
            autobalance_robot(emulated_gyroscope.getY());
        }

        // toggle_gripper(arm_joystick.getRawButton(1));

    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
        // double tiltAxis = robot_joystick.getY() * 15; // DEBUG: Emulate gyroscope
        // using joystick
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {

    }

    public void move_robot(double forward, double turn, double speed, boolean speed_isSliderInput) {
        if (speed_isSliderInput == true) {
            speed = max_drivePower * (Math.abs(speed - 1)) / 2;
        }
        robot.arcadeDrive(forward * speed, turn * speed); // Wilbert, Ryan
    }

    public double autobalance_robot(double source) {
        // YOU CAN CHANGE
        double maxAngle = 15.0;
        double minAngle = 2.5;
        double maximum_power = 0.2;
        
        // DO NOT CHANGE
        double autobalanceAxis = source;
        double output_power = 0;

        if (autobalanceAxis > maxAngle) {
            output_power = -maximum_power;
        } else if (autobalanceAxis < -maxAngle) {
            output_power = maximum_power;
        } else if (autobalanceAxis > minAngle) {
            output_power = -(maximum_power * ((autobalanceAxis - minAngle) / (maxAngle - minAngle)));
        } else if (autobalanceAxis < -minAngle) {
            output_power = (maximum_power * ((autobalanceAxis + minAngle) / (-maxAngle + minAngle)));
        } else {
            output_power = 0;
        };

        ab_publisher.set(autobalance_power); // display this in network tables for debugging
        return output_power;
    };

    public void move_robot_arm(boolean isPreset, double input, double target) {
        if (isPreset == false) {
        /* Breaking down the (if) statement:
          -> &&, ||
              * Boolean operators for and/or. && = and, || == or
          -> input > 0.1, input < -0.1
              * This acts as a deadzone for analog control of the robot's arm. 
              * The arm activates upon passing this threshold. 
          -> forwards_switch.get(), reverse_switch.get()
              * This gets a Boolean from the limit switch for when the arm reaches its forwards or reverse limit.
              * The arm moves up when the motor is going in reverse. The arm moves down when the motor is going forwards.
          -> limitSwitch_override == true
              * In the event any of the limit switches short in competiton and report only one value, this will force the
                motor to continue functioning instead of seizing because of a broken limit switch.
         */
            if (input > 0.1 /*Deadzone*/ && (forwards_switch.get() == false /*Limit Switch Not Pressed*/ || limitSwitch_override == true /*Limit Switches Disabled*/)) {
                motor_arm.set(arm_joystick.getY() * max_armPower);
            } else if (input < -0.1 && (reverse_switch.get() == false || limitSwitch_override == true)) {
                motor_arm.set(arm_joystick.getY() * max_armPower);
            } else if ((reverse_switch.get() == true || forwards_switch.get() == true) && limitSwitch_override == false) {
                motor_arm.set(0.0);
            } else {
                motor_arm.set(0.0);
            };

        } else if (isPreset == true) {
            motor_arm.setIdleMode(IdleMode.kCoast);
            while (Math.abs(arm_encoder.getPosition() - target) > 0.5) {
                // if 
                motor_arm.set(rotate_to.calculate(arm_encoder.getPosition(), target) * max_armPower);
            };
            motor_arm.setIdleMode(IdleMode.kBrake);
            motor_arm.set(0);
        };
    };

    public void toggle_gripper(boolean toggle) {
        double motor_default_speed = 0.075;
        if (toggle == true) {
            motor_gripper.set(motor_default_speed);
        } else if (toggle == false) {
            motor_gripper.set(-motor_default_speed);
        };
    };
};