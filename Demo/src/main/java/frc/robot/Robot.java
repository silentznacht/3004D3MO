// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Timer;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  // RIGHT MOTORS
    private final CANSparkMax rightMotor1 = new CANSparkMax(5, MotorType.kBrushed);
    private final CANSparkMax rightMotor2 = new CANSparkMax(6, MotorType.kBrushed);

  // LEFT MOTORS
    private final CANSparkMax leftMotor1 = new CANSparkMax(7, MotorType.kBrushed);
    private final CANSparkMax leftMotor2 = new CANSparkMax(8, MotorType.kBrushed);

  // UPPER-MECHANISM
    private final CANSparkMax inOutSysMotor = new CANSparkMax(9, MotorType.kBrushed);
    private final CANSparkMax hook = new CANSparkMax(10, MotorType.kBrushed);

  // Drive 
  private DifferentialDrive m_Drive = new DifferentialDrive(leftMotor1, rightMotor1);

  /*
   * Mechanism 1 (Upper): Sucks in & Out the ring
   * Mechanism 2 (Upper):  Hoops up
   */

  // CONTROLLERS
    private final XboxController userDriver = new XboxController(11);



  // Constants used to substitute powers or speeds
  static final double INTAKE_OUT_SPEED = 1.0; // percentage of feeder expelling note

  static final double INTAKE_IN_SPEED = -.4; // percentage of speed feeder taking in note


  static final int DRIVE_CURRENT_LIMIT_A = 60;  // drive train motor limit
  static final int DRIVE_ROTATE_SPEED = .3; 
  static final int DRIVE_POWER_SPEED = .60;
  static final int DRIVE_TURN_SPEED = .3;


  static final int FEEDER_CURRENT_LIMIT_A = 80; // feeder current limit

  static final double FEEDER_AMP_SPEED = .4; // Percent output for amp or drop note, configure based on polycarb bend

  static final int LAUNCHER_CURRENT_LIMIT_A = 80; //How many amps the launcher motor can use. lower to 60 if using neo

  static final double LAUNCHER_SPEED = 1.0; // percentage output for when lauchers is intaking and expelling note

  static final double LAUNCHER_AMP_SPEED = .17; // percentage output for scoring in amp or dropping note




// below if robot does not work try changing the initallizing code 

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    // sets motors to rest
    rightMotor1.set(0);
    rightMotor2.set(0);
    leftMotor1.set(0);
    leftMotor2.set(0);
    inOutSysMotor.set(0);
    hook.set(0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    rightMotor1.set(0);
    rightMotor2.set(0);
    leftMotor1.set(0);
    leftMotor2.set(0);
    inOutSysMotor.set(0);
    hook.set(0);
  }

  @Override
  public void teleopPeriodic() {

      // Intake
      if (userDriver.getRightBumperPressed()) { // powers intake for ring
          inOutSysMotor.set(FEEDER_AMP_SPEED); // prone to change according to constants || For Reakab: Make sure to adjust this intake to an according speed, just like a motor
      } else inOutSysMotor.set(0); // Sets intake to rest else

      // Hook
      if (userDriver.getLeftBumperPressed()) {
        hook.set(.3); // prone to change
      } else hook.set(0);


      // Drive 

      /*
        Kevin E:

        Potential Issues & Solutions:
          - If the movement of the robot is inversed or wonky, swap the x and y parameter (instead of .getY and then getX, swap it to .getX to .getY) 
            Or it could be one of the -userDrives, since im not there to physically test it, you must find which motor ( left or right ) will start negative or positive
            
          - If the rotator button (B) does not work, try substituting it with another button availiable in the Xbox library configuration

       */

      // Allows robot to move and rotate  
      m_Drive.arcadeDrive(userDriver.getY * DRIVE_POWER_SPEED, userDriver.getX * DRIVE_ROTATE_SPEED);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
