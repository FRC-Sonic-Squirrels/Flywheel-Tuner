/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_xboxController = new XboxController(0);
  private PowerDistributionPanel m_pdp = new PowerDistributionPanel();
  private int deviceID = 1;
  private int m_follow_deviceID = 0;    // CAN Id zero disables follow motor mode
  private boolean m_follow_motor_inverted = true;
  private double m_setPoint = 0;
  private long m_startTime_nanosec = 0;
  private double m_elapsedTime_sec = 0;
  private double overshot = 0;
  private double undershot = 0;
  private CANSparkMax m_motor;
  private CANSparkMax m_follow_motor = null;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  SendableChooser <String> mode_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {

    // PID coefficients (starting point)
    kP = 0.00003;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.00017; // 0.000015; 
    kMaxOutput = 1;
    kMinOutput = 0.0;  // -1
    maxRPM = 5700;

    initMotorController(deviceID, m_follow_deviceID, m_follow_motor_inverted);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("CAN Id", deviceID);
    SmartDashboard.putNumber("SetPoint (RPM)", m_setPoint);
    SmartDashboard.putNumber("Velocity (RPM)", m_encoder.getVelocity());
    SmartDashboard.putNumber("Total Current (Amp)", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total Power (W)", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Time to reach RPM", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (RPM)", 0.0);
    SmartDashboard.putNumber("Follow CAN Id", m_follow_deviceID);
    SmartDashboard.putBoolean("Invert Follow Motor", m_follow_motor_inverted);
    mode_chooser.addOption("Variable RPM (left stick)", "variable");
    mode_chooser.addOption("Fixed RPM (A, B, Y, X bottons)", "fixed");
    SmartDashboard.putData("Mode", mode_chooser);
  }

  private void initMotorController(int canId, int follow_canId, boolean follow_inverted) {

    deviceID = canId;

    // initialize motor
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters 
     * in the SPARK MAX to their factory default state. If no argument is passed, these 
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    if (m_follow_motor != null) {
      // If there was a follow motor before, reset it to factory defaults. (disable follow mode)
      m_follow_motor.restoreFactoryDefaults();
      // make sure motor is in coast mode, in case this motor is mechanically joined to the leaad motor
      m_follow_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    if (follow_canId != 0) {
      // configure follow motor
      m_follow_motor = new CANSparkMax(follow_canId, MotorType.kBrushless);
      m_follow_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
      m_follow_motor.follow(m_motor, follow_inverted);
    }
    else {
      m_follow_motor = null;
    }
    m_follow_deviceID = follow_canId;
    m_follow_motor_inverted = follow_inverted;

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();

    // Encoder object created to display position values
    m_encoder = m_motor.getEncoder();

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    int canId = (int) SmartDashboard.getNumber("CAN Id", 0);

    int follow_canId = (int) SmartDashboard.getNumber("Follow CAN Id", 0);
    boolean follow_inverted = (boolean) SmartDashboard.getBoolean("nvert Follow Motor", true);

    if ((canId != deviceID) || (follow_canId != m_follow_deviceID) || (follow_inverted != m_follow_motor_inverted)) {
      initMotorController(canId, follow_canId, follow_inverted);
    }

   // if PID coefficients on SmartDashboard have changed, write new values to controller
   if((p != kP)) { m_pidController.setP(p); kP = p; }
   if((i != kI)) { m_pidController.setI(i); kI = i; }
   if((d != kD)) { m_pidController.setD(d); kD = d; }
   if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
   if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
   if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max);
      kMinOutput = min; kMaxOutput = max;
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.ControlType.kDutyCycle
     *  com.revrobotics.ControlType.kPosition
     *  com.revrobotics.ControlType.kVelocity
     *  com.revrobotics.ControlType.kVoltage
     */
    double setPoint = m_setPoint;
    if (mode_chooser.getSelected() == "variable") {
      // left joystick set RPM setpoint
      setPoint =  m_xboxController.getY(Hand.kLeft) * maxRPM;
      if (Math.abs(setPoint) < 40) {
        // deadbanding. ignore really small joystick inputs
        setPoint = 0;
      }
      m_pidController.setReference(setPoint, ControlType.kVelocity);
    }
    else if (mode_chooser.getSelected() == "fixed") {
      // press A, B, Y, X buttons set speed
      // press Right Bumper to stop (set RPM to zero)
      if (m_xboxController.getAButtonPressed()) {
        setPoint = 1000;
        m_pidController.setReference(setPoint, ControlType.kVelocity);
      }
      else if (m_xboxController.getBButtonPressed()) {
        setPoint = 2000;
        m_pidController.setReference(setPoint, ControlType.kVelocity);
      }
      else if (m_xboxController.getYButtonPressed()) {
        setPoint = 3000;
        m_pidController.setReference(setPoint, ControlType.kVelocity);
      }
      else if (m_xboxController.getXButtonPressed()) {
        setPoint = 4000;
        m_pidController.setReference(setPoint, ControlType.kVelocity);
      }
      else if (m_xboxController.getBumperPressed(Hand.kRight)) {
        setPoint = 0;
        m_pidController.setReference(setPoint, ControlType.kVelocity);
      } 
    }

    if (m_setPoint != setPoint) {
      // set point changed, start a timer
      m_startTime_nanosec = System.nanoTime();
      m_elapsedTime_sec = 0;
      overshot = 0;
      undershot = 0;

      m_setPoint = setPoint;
    }

    double rpm = m_encoder.getVelocity();

    if (m_elapsedTime_sec == 0) {
      if (Math.abs(rpm - m_setPoint) < 10) {
          m_elapsedTime_sec = ((double)(System.nanoTime() - m_startTime_nanosec)) / 1000000000.0;
      }
    }

    double error = rpm - m_setPoint;

    if (m_elapsedTime_sec > 0) {
      // track max and min error after reaching target rpm
      if (error > overshot) {
        overshot = error;
      }
      if (error < undershot) {
        undershot = error;
      }
    }

    SmartDashboard.putNumber("SetPoint (RPM)", m_setPoint);
    SmartDashboard.putNumber("Velocity (RPM)", rpm);
    SmartDashboard.putNumber("Total Current (Amp)", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total Power (W)", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Time to reach RPM", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (RPM)", error);
  }
}
