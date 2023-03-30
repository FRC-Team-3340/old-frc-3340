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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser; // Chooser for autonomous
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // Debug use only on the computer

// WPILib Object Libraries and Inputs
import edu.wpi.first.wpilibj.drive.DifferentialDrive;                 // To use arcade drive
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
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
    private static final String kAutobalance = "Auto-balance Robot";
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
    private double drivePower[] = {0.25, 0.5, 0.7};
    private int selectPower = 0;
    private double max_drivePower = drivePower[selectPower]; // Base maximum power for driving the robot
    private double drive_turnRate = 0.75;;
    private double max_armPower = 0.2;
    private double gripperPower = 0.1;
    private boolean useLimitSwitches = true; // IF LIMIT SWITCH BREAKS, SET TO TRUE ON SMARTDASHBOARD OR HERE.
    
    private double presetRotations[] = {-5.0, -15.0, -30.0};
    private boolean arm_preset = false;
    
    // Autobalance Smart Dashboard compatibility
    public double abMaxAngle = 15.0;
    public double abMinAngle = 2.5;
    public double abMaxPower = 0.35;
    private EventLoop looper = new EventLoop();
    public BooleanEvent shiftGear = new BooleanEvent(looper, controller::getL3Button);

    // Logging and debugging utilities
    public NetworkTableInstance inst = NetworkTableInstance.getDefault();
    public NetworkTable stats_table = inst.getTable("SmartDashboard");
    public BooleanPublisher toggle_limit_switch = stats_table.getBooleanTopic("Use Limit Switches").publish();
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
    public double autonStartingRotation = 0.0;
    public double drive_distance = 0.0; // IN INCHES
    public double turn_angle = 0.0;
    public double arm_preset_value = 0.0;

    public boolean button_held = false;

    Thread camera_process;

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
        m_chooser.addOption("Autobalance", kAutobalance);

        SmartDashboard.putData("Auto choices", m_chooser);
        SmartDashboard.putData("PID", rotate_to);
        SmartDashboard.putNumber("Arm Position (Encoder)", Math.abs(arm_encoder.getPosition()));
        SmartDashboard.putNumber("Gripper Position (Encoder)", gripper_encoder.getPosition());

        toggle_limit_switch.set(useLimitSwitches);
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

        // Upon clicking the left stick, the robot will "shift gears", or switch through speeds.
        shiftGear.ifHigh(() -> {
        if (controller.getL3ButtonPressed() == true) {
            button_held = true;
        if (selectPower < 2) {
            selectPower++;
        } else {
            selectPower = 0;
        }
        max_drivePower = drivePower[selectPower];
        controller.setRumble(RumbleType.kBothRumble, max_drivePower);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        controller.setRumble(RumbleType.kBothRumble, 0);
        System.out.println("SHIFT GEAR");
        } 
        });
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
        autonStartingRotation = navX_gyro.getYaw();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kAutobalance:
                if (autonState == 0 && Math.abs(navX_gyro.getRoll()) > 2.5) {
                    autonState++;
                } else if (autonState == 1 && (navX_gyro.getRoll() < -2.5)) {
                    autonState++;
                }

                switch(autonState){
                    case 0:
                    robot.arcadeDrive(0.4, 0); 
                    break;
                    
                    case 1:
                    robot.arcadeDrive(0.2, 0); 
                    break;

                    case 2:
                    autobalance_power = autobalance_robot(navX_gyro.getRoll());
                    robot.arcadeDrive(autobalance_power, 0);
                    break;
                } 
                
                break;

            case kDefaultAuto:
            default:
                double autonRotations = drive_encoder.getPosition();
                double autonTurn = navX_gyro.getYaw();

                // distance * (42cts * 14gearRatio)/(6in*PI) = encoder Cts
                // Copy this for each step in autonomous period
                if (autonState == 0) {
                    drive_distance = 18; // 18 in drive distance for this step
                    if (autonRotations - autonStartingPostion > (drive_distance * (42 * 14) / (6 * Math.PI))) {
                        autonState++;
                        autonStartingPostion = drive_encoder.getPosition();
                    } else {
                        robot.arcadeDrive(-0.25, 0); // Speed for this step
                        System.out.printf("%-20f%f%n", autonStartingPostion, (drive_distance * (42 * 14) / (6 * Math.PI)), autonRotations);
                    }
                }
                if (autonState == 1) {
                    drive_distance = 36;
                    if (autonRotations - autonStartingPostion > (drive_distance * (42 * 14) / (6 * Math.PI))) {
                        autonState++;
                        autonStartingPostion = drive_encoder.getPosition();
                    } else {
                        robot.arcadeDrive(0.25, 0);
                        System.out.printf("%-20f%f%f%n", autonStartingPostion, (drive_distance * (42 * 14) / (6 * Math.PI)), autonRotations);
                    }
                }
                if (autonState == 2) {
                    robot.arcadeDrive(0, 0);
                }

                // This is for a rotation 
                if (autonState == 3){
                    turn_angle = 90.0;
                    if (autonTurn - autonStartingRotation > turn_angle){
                        autonState++;
                        autonStartingRotation = navX_gyro.getYaw();
                    } else {
                        robot.arcadeDrive(0, 0.5); // CHECK direction
                        System.out.printf("%-20f%f%f%n", autonTurn, autonStartingRotation, turn_angle);
                    }


                }
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
        useLimitSwitches = SmartDashboard.getBoolean("Forward Limit Enabled", true);

        // Driving the robot, allowing support for twisting and moving stick left and right.
        move_robot(controller.getLeftY(), controller.getLeftX(), max_drivePower);
        move_robot((-controller.getL2Axis() + controller.getR2Axis()) * max_drivePower, controller.getLeftX(), max_drivePower);
 
        // Presets for the arm
        if (controller.getPOV() <= 180 && controller.getPOV() != -1 && arm_preset != true) {
            arm_preset = true;
            if (controller.getPOV() == 0) {
                arm_preset_value = presetRotations[2];
            } else if (controller.getPOV() == 90) {
                arm_preset_value = presetRotations[1];
            } else if (controller.getPOV() == 180) {
                arm_preset_value = presetRotations[0];
            }  
            System.out.printf("Set rotation to %f%n", arm_preset_value);
        }

        move_robot_arm(controller.getRightY(), arm_preset_value);
        toggle_gripper(controller.getR1Button(), controller.getL1Button());
        looper.poll(); // Allows for shifting gears to work :)
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
        useLimitSwitches = SmartDashboard.getBoolean("Forward Limit Enabled", true);
        double movement = (-controller.getL2Axis() + controller.getR2Axis()) * max_drivePower;
        move_robot(movement, controller.getLeftX(), max_drivePower);

        // Presets for the arm
        if (controller.getPOV() <= 180 && controller.getPOV() != -1 && arm_preset == true) {
            arm_preset = true;
            if (controller.getPOV() == 0) {
                arm_preset_value = presetRotations[2];
            } else if (controller.getPOV() == 90) {
                arm_preset_value = presetRotations[1];
            } else if (controller.getPOV() == 180) {
                arm_preset_value = presetRotations[0];
            }  
            System.out.printf("Set rotation to %f%n", arm_preset_value);
        }
        
        move_robot_arm(controller.getRightY(), arm_preset_value);
        toggle_gripper(controller.getR1Button(), controller.getL1Button());
        looper.poll();

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
        robot.arcadeDrive(move * gear, turn * gear);
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
            if ((reverse_switch.get() == true || forwards_switch.get() == true )&& useLimitSwitches != false) {
                motor_arm.set(0.0); // stops if limit is hit
            } else if (Math.abs(input) > deadzone) {
                if (reverse_switch.get() == true && input < 0) {
                    motor_arm.set( -Math.abs(input) * max_armPower); // allows for movement in one direction
                } else if (forwards_switch.get() == true && input > 0){
                    motor_arm.set(Math.abs(input) * max_armPower); // allows for movement in one direction
                } else {
                    motor_arm.set(input * max_armPower); // move according to joystick
                }
            } else {
                motor_arm.set(-idle_power); // idle if no input
            }
        }

        

    }
    // boop
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