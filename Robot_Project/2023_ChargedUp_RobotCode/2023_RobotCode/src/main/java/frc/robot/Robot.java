// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Also some lines can be shortened to aid in readability Ex. 261, 263, 265
 * Might also wanna shorten the excessive comment code or seperate it better
 */
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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;

// Network Table
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;

// Imports for sensors, motors, and inputs - comment what each import is for
import com.kauailabs.navx.frc.AHRS; // navX-MXP IMU that has a useful gyroscope
import com.revrobotics.CANSparkMax; // Spark MAX motor controller
import edu.wpi.first.wpilibj.Servo  ;
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
    private PIDController rotate_to = new PIDController(0.1, 0.01, 0.5);

    // LIMIT SWITCHES
    public DigitalInput reverse_switch = new DigitalInput(0);
    public DigitalInput forwards_switch = new DigitalInput(1);

    // SERVO
    public Servo gripperServo = new Servo(0);

    // GLOBAL FOR SMART DASHBOARD.
    private double max_drivePower = 0.5; // Base maximum power for driving the robot
    private double max_armPower = 0.15;
    private double gripperPower = 0.1;
    private boolean limitSwitch_override = false; // IF LIMIT SWITCH BREAKS, SET TO TRUE ON SMARTDASHBOARD OR HERE.
    
    // Autobalance Smart Dashboard compatibility
    public double maxAngle = 15.0;
    public double minAngle = 2.5;
    public double maximum_power = 0.35;
    
    // Logging and debugging utilities
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable stats_table = inst.getTable("SmartDashboard");
    public BooleanPublisher toggle_limit_switch = stats_table.getBooleanTopic("Disable Limit Switches").publish();
    public DoublePublisher armPower = stats_table.getDoubleTopic("Arm Power").publish();
    public DoublePublisher drivePower = stats_table.getDoubleTopic("Drive Power").publish();
    public DoublePublisher ab_maxAngle = stats_table.getDoubleTopic("Autobalance - Maximum Angle").publish();
    public DoublePublisher gripperPublisher = stats_table.getDoubleTopic("Gripper Power").publish();
    public DoublePublisher ab_minAngle = stats_table.getDoubleTopic("Autobalance - Minimum Angle").publish();
    public DoublePublisher ab_maxPower = stats_table.getDoubleTopic("Autobalance - Maximum Power").publish();
    public DoublePublisher ab_axisMeausre = stats_table.getDoubleTopic("Gyroscope Axis (Autobalance)").publish();

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
        SmartDashboard.putData("PID", rotate_to);
        SmartDashboard.putNumber("Arm Position (Encoder)", arm_encoder.getPosition());
        SmartDashboard.putNumber("Gripper Position (Encoder)", gripper_encoder.getPosition());

        toggle_limit_switch.set(limitSwitch_override);
        drivePower.set(max_drivePower);
        ab_maxAngle.set(maxAngle);
        ab_minAngle.set(minAngle);
        gripperPublisher.set(gripperPower);
        armPower.set(max_armPower);
        ab_maxPower.set(maximum_power);
        ab_axisMeausre.set(navX_gyro.getRoll());


        // Initialize robot arm and gripper
        motor_arm.restoreFactoryDefaults();
        motor_gripper.restoreFactoryDefaults();

        
        // motor_gripper.setIdleMode(IdleMode.kCoast);
        // motor_gripper.setSoftLimit(SoftLimitDirection.kReverse, -25);

        CameraServer.startAutomaticCapture();
    }

    @Override
    public void robotPeriodic() {

    }
    @Override
    public void autonomousInit() {
        m_autoSelected = m_chooser.getSelected();
        navX_gyro.calibrate();
        gripperServo.close();
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
                while (Math.abs(navX_gyro.getRoll()) < 2.5) {
                    move_robot(.2, 0, 1, false); 
                }
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
                    } 
                } 
                break;
            case kDefaultAuto:
            default:
                RelativeEncoder drive_encoder = motorL_front.getEncoder();
                double starting_distance = drive_encoder.getPosition();

                while (drive_encoder.getPosition() < starting_distance + 100) {
                    move_robot(1, 0, .25, false);
                }
                starting_distance = drive_encoder.getPosition();
                while (drive_encoder.getPosition() > starting_distance - 200) {
                    move_robot(-1, 0, .25, false); 
                }
                break;
        }
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        gripper_encoder.setPosition(0);
        // motor_gripper.enableSoftLimit(SoftLimitDirection.kReverse, true);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        limitSwitch_override = SmartDashboard.getBoolean("Forward Limit Enabled", false);

        // Driving the robot, allowing support for twisting and moving stick left and right.
        if (Math.abs(robot_joystick.getX()) < Math.abs(robot_joystick.getZ())){
            move_robot(robot_joystick.getY(), robot_joystick.getZ(), robot_joystick.getRawAxis(3), true);
        } else {
            move_robot(robot_joystick.getY(), robot_joystick.getX(), robot_joystick.getRawAxis(3), true);
        }   
        
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

        toggle_gripper(arm_joystick.getRawButton(1), arm_joystick.getRawButton(2));
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // motor_gripper.enableSoftLimit(SoftLimitDirection.kReverse, false);
    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
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

        move_robot(robot_joystick.getY(), robot_joystick.getZ(), robot_joystick.getRawAxis(3), true);

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
        // DO NOT CHANGE
        double autobalanceAxis = source;
        double output_power = 0;

        if (autobalanceAxis > maxAngle) {
            output_power = -maximum_power;
        } else if (autobalanceAxis < -maxAngle) {
            output_power = maximum_power/1.25;
        } else if (autobalanceAxis > minAngle) {
            output_power = -(maximum_power * ((autobalanceAxis - minAngle) / (maxAngle - minAngle)));
        } else if (autobalanceAxis < -minAngle) {
            output_power = (maximum_power/1.25 * ((autobalanceAxis + minAngle) / (-maxAngle + minAngle)));
        } else {
            output_power = 0;
        }

        return output_power;
    }

    public void move_robot_arm(boolean isPreset, double input, double target) {
        if (isPreset == false) {
            if (input > 0.1 /*Deadzone*/ && (forwards_switch.get() == false /*Limit Switch Not Pressed*/ || limitSwitch_override == true /*Limit Switches Disabled*/)) {
                motor_arm.set(arm_joystick.getY() * max_armPower);
            } else if (input < -0.1 && (reverse_switch.get() == false || limitSwitch_override == true)) {
                motor_arm.set(arm_joystick.getY() * max_armPower);
            } else if ((reverse_switch.get() == true || forwards_switch.get() == true) && limitSwitch_override == false) {
                motor_arm.set(0.0); 
            } else {
                motor_arm.set(0.05);
            }

        } else if (isPreset == true) {
            motor_arm.setIdleMode(IdleMode.kCoast);
            while (Math.abs(arm_encoder.getPosition() - target) > 0.5) {
                
                if (rotate_to.calculate(arm_encoder.getPosition(), target) > max_armPower) {
                    motor_arm.set(max_armPower);
                } else {
                    motor_arm.set(rotate_to.calculate(arm_encoder.getPosition(), target) * max_armPower);
                }
            }
            motor_arm.setIdleMode(IdleMode.kBrake);
            motor_arm.set(0);
        }
    }

    public void toggle_gripper(boolean close, boolean open) {
        if (close == true && open == false) {
            motor_gripper.set(gripperPower);
        } else if (open == true && close == false) {
            motor_gripper.set(-gripperPower);
        } else {
          motor_gripper.set(0);
        }
    }
}