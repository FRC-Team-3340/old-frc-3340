// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
    * Migrating to PS5 single-player control. 
 */


package frc.robot;

// WPILib Imports

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot; // Robot Type
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // Chooser for autonomous
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Debug use only on the computer

// WPILib Object Libraries and Inputs
import edu.wpi.first.wpilibj.drive.DifferentialDrive;                 // To use arcade drive
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;       // Conjoins two motors as one
import edu.wpi.first.wpilibj.DigitalInput;                            // Limit Switch interface for the robot's arm
import edu.wpi.first.wpilibj.Joystick;                                // Joystick interface for controlling the robot
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;

// Network Table
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;

import java.lang.reflect.Array;

// Imports for sensors, motors, and inputs - comment what each import is for
import com.kauailabs.navx.frc.AHRS; // navX-MXP IMU that has a useful gyroscope
import com.revrobotics.CANSparkMax; // Spark MAX motor controller
import edu.wpi.first.wpilibj.Servo  ;
import com.revrobotics.RelativeEncoder;
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
    
    // Uncomment which one you need.
    private PS4Controller controller = new PS4Controller(0); // Create joystick interface object
    // private XboxController controller = new XboxController(0);

    /* All of these have been merged to one controller, preferably a PS4 controller */
    // private Joystick robot_joystick = new Joystick(0);
    // private Joystick arm_joystick = new Joystick(1);
    // private Joystick emulated_gyroscope = new Joystick(2); // ignore this

    public RelativeEncoder arm_encoder = motor_arm.getEncoder();
    public RelativeEncoder gripper_encoder = motor_gripper.getEncoder();
    public RelativeEncoder drive_encoder = motorL_front.getEncoder();


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
    private double drive_turnRate = 0.75;
    public int current_gear = 1;
    private double max_armPower = 0.2;
    private double gripperPower = 0.1;
    private boolean limitSwitch_override = true; // IF LIMIT SWITCH BREAKS, SET TO TRUE ON SMARTDASHBOARD OR HERE.
    
    private double presetRotations[] = {-5.0, -15.0, -30.0};
    private boolean arm_preset = false;
    
    // Autobalance Smart Dashboard compatibility
    public double abMaxAngle = 15.0;
    public double abMinAngle = 2.5;
    public double abMaxPower = 0.35;
    

    // Logging and debugging utilities
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable stats_table = inst.getTable("SmartDashboard");
    public BooleanPublisher toggle_limit_switch = stats_table.getBooleanTopic("Disable Limit Switches").publish();
    public DoublePublisher nt_armPower = stats_table.getDoubleTopic("Arm Power").publish();
    public DoublePublisher nt_drivePower = stats_table.getDoubleTopic("Drive Power").publish();
    public DoublePublisher nt_abMaxAngle = stats_table.getDoubleTopic("Autobalance - Maximum Angle").publish();
    public DoublePublisher nt_gripper = stats_table.getDoubleTopic("Gripper Power").publish();
    public DoublePublisher nt_abMinAngle = stats_table.getDoubleTopic("Autobalance - Minimum Angle").publish();
    public DoublePublisher nt_abPower = stats_table.getDoubleTopic("Autobalance - Maximum Power").publish();
    public DoublePublisher ab_axisMeausre = stats_table.getDoubleTopic("Gyroscope Axis (Autobalance)").publish();

    public double autobalance_power;
    public int lastPressed = 0;

    public int autonState = 0;
    public double autonStartingPostion = 0.0;
    public double drive_distance; // IN INCHES
    public double preset = 0;


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
        nt_drivePower.set(max_drivePower);
        nt_abMaxAngle.set(abMaxAngle);
        nt_abMinAngle.set(abMinAngle);
        nt_gripper.set(gripperPower);
        nt_armPower.set(max_armPower);
        nt_abPower.set(abMaxPower);
        ab_axisMeausre.set(navX_gyro.getRoll());


        // Initialize robot arm and gripper
        motor_arm.restoreFactoryDefaults();
        motor_gripper.restoreFactoryDefaults();

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
        drive_encoder.setPosition(0);
        autonState = 0;   
        autonStartingPostion = drive_encoder.getPosition();
        
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
                if (Math.abs(navX_gyro.getRoll()) < 2.5) {
                    robot.arcadeDrive(0.25, 0); 
                }
                else if (balanced == false) {
                    autobalance_power = autobalance_robot(navX_gyro.getRoll());
                    robot.arcadeDrive(autobalance_power, 0);
                    if (autobalance_power == 0) {
                        time_balanced++;
                    }
                    if (time_balanced == 10000) {
                        System.out.println("Robot is balanced :)");
                        balanced = true;
        
                    } 
                } 
                break;
            case kDefaultAuto:
            default:
                double autonRotations = Math.abs(drive_encoder.getPosition());
                //14:1 6in wheel
                // drive_distance measures inches.
                if (autonState == 0) {
                    drive_distance = 18;
                    if (autonRotations > ((2.35*(42/14) * drive_distance / (6 * Math.PI)))) {
                        autonState++;
                        autonStartingPostion = Math.abs(drive_encoder.getPosition());
                    } else {
                        robot.arcadeDrive(-0.25, 0);
                    }
                }
                if (autonState == 1) {
                    drive_distance = 36;
                    if (autonRotations - autonStartingPostion < ((2.35*(42/14) * drive_distance / (6 * Math.PI)))) {
                        autonState++;
                        autonStartingPostion = Math.abs(drive_encoder.getPosition());
                    } else {
                        robot.arcadeDrive(0.25, 0);
                    }
                } else {
                    robot.arcadeDrive(0, 0);
                }
                /* System.out.println((42*6)*3.14 *18/14.0);
                // if (autonState == 0 && drive_encoder.getPosition() > (3 *(42*6)*3.    *18/14.0)) {
                //     autonState++;
                // }
                // if (autonState == 1 && drive_encoder.getPosition() < (42*6)*3.14 *32/14.0) {
                //     autonState++;                    
                */ 
        }
    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        gripper_encoder.setPosition(0);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        limitSwitch_override = SmartDashboard.getBoolean("Forward Limit Enabled", false);

        // Driving the robot, allowing support for twisting and moving stick left and right.
        move_robot(controller.getLeftY(), controller.getLeftX(), current_gear);

        
        // // Redesigned preset code: detects if any of the buttons are pressed
        // if (arm_joystick.getRawButton(8) == true || 
        //     arm_joystick.getRawButton(10) == true || 
        //     arm_joystick.getRawButton(12) == true) {
        //         arm_preset = true; // Upon the button being pressed, it tells the robot to ignore stick input.
        //         // The button pressed determines where the robot should go. 
        //         if (arm_joystick.getRawButton(8) == true) {
        //             presetRotation = -30;
        //         } else if (arm_joystick.getRawButton(10) == true) {
        //             presetRotation = -15;
        //         } else if (arm_joystick.getRawButton(12) == true) {
        //             presetRotation = -5;
        //         }
        // }

        // Preset code for single controller
        // if (controller.getTopPressed() == true || controller.get)

        // move_robot_arm(controller.getRightY(), presetRotation);
        toggle_gripper(controller.getR1Button(), controller.getL1Button());
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {

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
        double movement = (-controller.getL2Axis() + controller.getR2Axis()) * max_drivePower;
        move_robot(movement, controller.getLeftX(), current_gear);

        /*
        if (arm_joystick.getRawButton(8) == true || 
            arm_joystick.getRawButton(10) == true || 
            arm_joystick.getRawButton(12) == true) {
                if (arm_joystick.getRawButton(8) == true) {
                    presetRotation = -30;
                } else if (arm_joystick.getRawButton(10) == true) {
                    presetRotation = -15;
                } else if (arm_joystick.getRawButton(12) == true) {
                    presetRotation = -5;
                }
        }
        */
        if (controller.getPOV() <= 180) {
            arm_preset = true;
            if (controller.getPOV() == 0) {
                preset = presetRotations[0];
            } else if (controller.getPOV() == 90) {
                preset = presetRotations[1];
            } else if (controller.getPOV() == 180) {
                preset = presetRotations[2];
            }  
        }

        move_robot_arm(controller.getRightY(), preset);
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

    // public void move_robot(double forward, double turn, double speed, boolean speed_isSliderInput) {
    //     if (speed_isSliderInput == true) {
    //         speed = max_drivePower * (Math.abs(speed - 1)) / 2;
    //     }
    //     robot.arcadeDrive(forward * speed, drive_turnRate * turn * speed); // Wilbert, Ryan
    // }

    // This incorporates shifting gear instead of the slider.
    public void move_robot(double move, double turn, double gear) {
        double speed = max_drivePower * gear;

        robot.arcadeDrive(move * speed, turn * speed);
    }

    public double autobalance_robot(double source) {
        // DO NOT CHANGE
        double autobalanceAxis = source;
        double output_power = 0;

        if (autobalanceAxis > abMaxAngle) {
            output_power = -abMaxPower;
        } else if (autobalanceAxis < -abMaxAngle) {
            output_power = abMaxPower/1.25;
        } else if (autobalanceAxis > abMinAngle) {
            output_power = -(abMaxPower * ((autobalanceAxis - abMinAngle) / (abMaxAngle - abMinAngle)));
        } else if (autobalanceAxis < -abMinAngle) {
            output_power = (abMaxPower/1.25 * ((autobalanceAxis + abMinAngle) / (-abMaxAngle + abMinAngle)));
        } else {
            output_power = 0;
        }

        return output_power;
    }

    public void move_robot_arm(double input, double target) {
        double preset_margin = 1; // Set the margin of error for the presets
        /*Example: If your preset is set to go to -36, it should stop between -35.0 and -37.0. */
        double deadzone = 0.1;      // Set joystick deadzone to prevent drift
        double idle_power = 0.05;   // Set power enough so that the arm holds up but does not sag.

        if (arm_preset == true && (arm_encoder.getPosition() < target + preset_margin && arm_encoder.getPosition() > target - preset_margin)) {
            if (rotate_to.calculate(arm_encoder.getPosition(), target) > max_armPower) {
                motor_arm.set(max_armPower);
            } else if (rotate_to.calculate(arm_encoder.getPosition(), target) < -max_armPower) {
                    motor_arm.set(-max_armPower);
            } else {
                motor_arm.set(rotate_to.calculate(arm_encoder.getPosition(), target) * max_armPower);
            }
        } else {
            arm_preset = false;
            if ((reverse_switch.get() == true || forwards_switch.get() == true) && limitSwitch_override == false) {
                motor_arm.set(0.0); 
                // System.out.println(input);
            } else if (Math.abs(input) > deadzone) {
                motor_arm.set(input * max_armPower);
            } else {
                motor_arm.set(-idle_power); 
            }
        }
    }

    public void toggle_gripper(boolean close, boolean open) {
        if (close == true && open == false) {

            try{
                Thread.sleep(100);
            } catch (InterruptedException ex){
                ex.printStackTrace();
            }

            motor_gripper.set(gripperPower);
        } else if (open == true && close == false) {

            try{
                Thread.sleep(100);
            } catch (InterruptedException ex){
                ex.printStackTrace();
            }

           motor_gripper.set(-gripperPower);
        } else {
          motor_gripper.set(0);
        }
    }
}