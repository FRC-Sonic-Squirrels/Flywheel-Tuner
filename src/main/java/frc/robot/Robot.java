/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2021 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_xboxController = new XboxController(0);
  private PowerDistribution m_pdp = new PowerDistribution();
  private int deviceID = 1;
  private int m_follow_deviceID = 0;    // CAN Id zero disables follow motor mode
  private boolean m_follow_motor_inverted = true;
  private double m_setPoint = 0;
  private long m_startTime_nanosec = 0;
  private double m_elapsedTime_sec = 0;
  private double overshot = 0;
  private double undershot = 0;
  private WPI_TalonFX m_motor;
  private WPI_TalonFX m_follow_motor = null;
  // TalonFX doesn't use a separate pid controller object
  private TalonFXSensorCollection m_encoder;
  private boolean m_invert_motor = true;
  private SlewRateLimiter m_rateLimiter;
  private double m_rate_RPMpersecond;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  SendableChooser <String> mode_chooser = new SendableChooser<>();

  final int kPIDLoopIdx = 0;
  final int kTimeoutMs = 30;

  // FalconFX reports velocity in counts per 100ms
  // 1 revolution = 2048 counts
  // 1 minutes = 60 * 10 * 100ms
  // conversion is  600  / 2048
  public double ticks2RPm = 600.0 / 2048.0;

  @Override
  public void robotInit() {

    // PID coefficients (starting point)
    // Small initial kFF and kP values, probably just big enough to do *something* 
    // and *probably* too small to overdrive an untuned system.
    kFF = 0.02;
    kP = 0.04;
    kI = 0;
    kD = 0;
    kIz = 0;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    maxRPM = 6300;     // free speed of Falcon 500 is listed as 6380
    m_rate_RPMpersecond = 1e10;    // 10 million effectively disables rate limiting

    m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_setPoint);

    initMotorController(deviceID, m_invert_motor, m_follow_deviceID, m_follow_motor_inverted);

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
    SmartDashboard.putNumber("Velocity (RPM)", m_encoder.getIntegratedSensorVelocity() * ticks2RPm );
    SmartDashboard.putNumber("Total Current (Amp)", m_motor.getStatorCurrent());
    SmartDashboard.putNumber("Total Power (W)", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Time to reach RPM", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (RPM)", 0.0);
    SmartDashboard.putNumber("Follow CAN Id", m_follow_deviceID);
    SmartDashboard.putBoolean("Invert Follow Motor", m_follow_motor_inverted);
    SmartDashboard.putBoolean("Invert Lead Motor", m_invert_motor);
    mode_chooser.addOption("Fixed RPM (A, B, Y, X buttons)", "fixed");
    mode_chooser.addOption("Variable RPM (left stick)", "variable");
    SmartDashboard.putData("Mode", mode_chooser);
    SmartDashboard.putNumber("Applied Output", 0.0);
    SmartDashboard.putNumber("Ramp Rate (RPM/s)", m_rate_RPMpersecond);

  }

  private void initMotorController(int canId, boolean invert_motor, int follow_canId, boolean follow_inverted) {

    deviceID = canId;
    m_invert_motor = invert_motor;

    // initialize motor
    m_motor = new WPI_TalonFX(deviceID);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters 
     * in the SPARK MAX to their factory default state. If no argument is passed, these 
     * parameters will not persist between power cycles
     */
    m_motor.configFactoryDefault();
    m_motor.setNeutralMode(NeutralMode.Coast);
    m_motor.setInverted(m_invert_motor);
    m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    if (m_follow_motor != null) {
      // If there was a follow motor before, reset it to factory defaults. (disable follow mode)
      m_follow_motor.configFactoryDefault();
      // make sure motor is in coast mode, in case this motor is mechanically joined to the lead motor
      m_follow_motor.setNeutralMode(NeutralMode.Coast);
    }

    if (follow_canId != 0) {
      // configure follow motor
      m_follow_motor = new WPI_TalonFX(follow_canId);
      m_follow_motor.setNeutralMode(NeutralMode.Coast);
      m_follow_motor.follow(m_motor, FollowerType.PercentOutput);
      // always spin opposite of the lead motor
      m_follow_motor.setInverted(InvertType.OpposeMaster);
    }
    else {
      m_follow_motor = null;
    }
    m_follow_deviceID = follow_canId;
    m_follow_motor_inverted = follow_inverted;

    /**
     * TalonFX sets PID values directly in the WPI_TalonFX object.
     */

    // Encoder object created to display position values
    m_encoder = m_motor.getSensorCollection();

    // set PID coefficients
    m_motor.config_kF(kPIDLoopIdx, kFF, kTimeoutMs);
		m_motor.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		m_motor.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		m_motor.config_kD(kPIDLoopIdx, kD, kTimeoutMs);
    m_motor.config_IntegralZone(kPIDLoopIdx, kIz, kTimeoutMs);
    m_motor.configNominalOutputForward(0, kTimeoutMs);
    m_motor.configNominalOutputReverse(0, kTimeoutMs);
		m_motor.configPeakOutputForward(1, kTimeoutMs);
		m_motor.configPeakOutputReverse(-1, kTimeoutMs);

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
    boolean invert_motor = SmartDashboard.getBoolean("Invert Lead Motor", m_invert_motor);
    int follow_canId = (int) SmartDashboard.getNumber("Follow CAN Id", 0);
    boolean follow_inverted = (boolean) SmartDashboard.getBoolean("Invert Follow Motor", true);

    if ((canId != deviceID) || (invert_motor != m_invert_motor) || (follow_canId != m_follow_deviceID)
        || (follow_inverted != m_follow_motor_inverted)) {
      initMotorController(canId, invert_motor, follow_canId, follow_inverted);

      // Reset RPM to zero if we change anything about the motor configuration. (safety first!)
      m_setPoint = 0;
    }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_motor.config_kP(kPIDLoopIdx, p, kTimeoutMs); kP = p; }
    if((i != kI)) { m_motor.config_kI(kPIDLoopIdx, i, kTimeoutMs); kI = i; }
    if((d != kD)) { m_motor.config_kD(kPIDLoopIdx, d, kTimeoutMs); kD = d; }
    if((iz != kIz)) { m_motor.config_IntegralZone(kPIDLoopIdx, iz, kTimeoutMs); kIz = iz; }
    if((ff != kFF)) { m_motor.config_kF(kPIDLoopIdx, ff, kTimeoutMs);; kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_motor.configPeakOutputForward(max, kTimeoutMs);
      m_motor.configPeakOutputReverse(min, kTimeoutMs);
      kMinOutput = min; kMaxOutput = max;
    }
    
    double ramprate = SmartDashboard.getNumber("Ramp Rate (RPM/s)", 0);
    if (ramprate != m_rate_RPMpersecond) {
      m_rateLimiter = new SlewRateLimiter(ramprate, m_setPoint);
      m_rate_RPMpersecond = ramprate;
      SmartDashboard.putNumber("Ramp Rate (RPM/s)", m_rate_RPMpersecond);
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
      // left joystick set RPM set point
      setPoint =  m_xboxController.getLeftY() * maxRPM;
      if (Math.abs(setPoint) < 40) {
        // dead banding. ignore really small joystick inputs
        setPoint = 0;
      }
    }
    else if (mode_chooser.getSelected() == "fixed") {
      // press A, B, Y, X buttons set speed
      // press Right Bumper to stop (set RPM to zero)
      if (m_xboxController.getAButtonPressed()) {
        setPoint = 1000;
      }
      else if (m_xboxController.getBButtonPressed()) {
        setPoint = 2000;
      }
      else if (m_xboxController.getYButtonPressed()) {
        setPoint = 3000;
      }
      else if (m_xboxController.getXButtonPressed()) {
        setPoint = 4000;
      }
      else if (m_xboxController.getRightBumperPressed()) {
        setPoint = 0;
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

    double rpm = m_encoder.getIntegratedSensorVelocity() * ticks2RPm;

    if (m_elapsedTime_sec == 0) {
      if (Math.abs(rpm - m_setPoint) < 50) {
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

    // Calculate and set new reference RPM
    double reference_setpoint = m_rateLimiter.calculate(setPoint);
    if (setPoint == 0) {
       // when we hit  stop, stop immediately. (safety!)
      reference_setpoint = 0;
      m_rateLimiter.reset(0);
    }
    
    m_motor.set(ControlMode.Velocity, reference_setpoint / ticks2RPm);

    SmartDashboard.putNumber("SetPoint (RPM)", reference_setpoint);  // was m_setpoint
    SmartDashboard.putNumber("Velocity (RPM)", rpm);
    SmartDashboard.putNumber("Total Current (Amp)", m_pdp.getTotalCurrent());
    SmartDashboard.putNumber("Total Power (W)", m_pdp.getTotalPower());
    SmartDashboard.putNumber("Time to reach RPM", m_elapsedTime_sec);
    SmartDashboard.putNumber("Overshot", overshot);
    SmartDashboard.putNumber("Undershot", undershot);
    SmartDashboard.putNumber("Error (RPM)", error);
    SmartDashboard.putNumber("Applied Output", m_motor.getMotorOutputVoltage());
  }
}
