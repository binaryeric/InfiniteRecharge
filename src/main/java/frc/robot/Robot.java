/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * The VM is configured to automatically run this class. If you change the name
 * of this class or the package after creating this project, you must also
 * update the build.gradle file in the project.
 */
public class Robot extends RobotBase {
  
  private XboxController xbox = new XboxController(0);
  private VictorSPX left = new VictorSPX(1);
  private VictorSPX right = new VictorSPX(2);
  
  private VictorSPX conveyors[] = {new VictorSPX(3), new VictorSPX(4)};

  private VictorSPX intake775 = new VictorSPX(5);
  private VictorSPX intakeFlip = new VictorSPX(6); 

  public void robotInit() {
  }

  public void disabled() {
  }

  public void autonomous() {
  }

  public void teleop() {
    while(isEnabled() == true) {
      // Intake Motor: Use Right Trigger
      if(xbox.getRawAxis(3) > 0.4) {
        intake775.set(ControlMode.Current, xbox.getRawAxis(4));
      } else {
        intake775.set(ControlMode.Current, 0);
      }
      // Conveyors motors: Use Left trigger
      if(xbox.getRawAxis(5) > 0.4) {
        setConveyors(xbox.getRawAxis(5));
      } else {
        setConveyors(0);
      }
      // Intake A for out Y for in
      if(xbox.getAButton() == true) {
        intakeFlip.set(ControlMode.Current, 0.4);
      } else if(xbox.getYButton() == true) {
        intakeFlip.set(ControlMode.Current, -0.4);
      } else {
        intakeFlip.set(ControlMode.Current, 0);
      }

      // lastly arcade drive:
      arcadeDrive(xbox.getRawAxis(1), xbox.getRawAxis(4), true);
    }
  }

  public void test() {
  }

  // methods
  public void setConveyors(double val) {
    conveyors[0].set(ControlMode.Current, val);
    conveyors[1].set(ControlMode.Current, -val);
  }

  public double deadZone(double v) {
    if(Math.abs(v) < 0.1) {
      return 0;
    }
    return v;
  }

  public void arcadeDrive(double moveValue, double rotateValue, boolean squaredInputs) {
    // local variables to hold the computed PWM values for the motors

    moveValue = deadZone(moveValue);
    rotateValue = deadZone(rotateValue);

    double leftMotorSpeed;
    double rightMotorSpeed;

    // square the inputs (while preserving the sign) to increase fine control
    // while permitting full power
    if (squaredInputs) {
      // square the inputs (while preserving the sign) to increase fine control
      // while permitting full power
      moveValue = Math.copySign(moveValue * moveValue, moveValue);
      rotateValue = Math.copySign(rotateValue * rotateValue, rotateValue);
    }

    if (moveValue > 0.0) {
      if (rotateValue > 0.0) {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = Math.max(moveValue, rotateValue);
      } else {
        leftMotorSpeed = Math.max(moveValue, -rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      }
    } else {
      if (rotateValue > 0.0) {
        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
        rightMotorSpeed = moveValue + rotateValue;
      } else {
        leftMotorSpeed = moveValue - rotateValue;
        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
      }
    }  
    left.set(ControlMode.Current, leftMotorSpeed);
    right.set(ControlMode.Current, rightMotorSpeed);
  }

  private volatile boolean m_exit;

  @SuppressWarnings("PMD.CyclomaticComplexity")
  @Override
  public void startCompetition() {
    robotInit();

    // Tell the DS that the robot is ready to be enabled
    HAL.observeUserProgramStarting();

    while (!Thread.currentThread().isInterrupted() && !m_exit) {
      if (isDisabled()) {
        m_ds.InDisabled(true);
        disabled();
        m_ds.InDisabled(false);
        while (isDisabled()) {
          m_ds.waitForData();
        }
      } else if (isAutonomous()) {
        m_ds.InAutonomous(true);
        autonomous();
        m_ds.InAutonomous(false);
        while (isAutonomous() && !isDisabled()) {
          m_ds.waitForData();
        }
      } else if (isTest()) {
        LiveWindow.setEnabled(true);
        Shuffleboard.enableActuatorWidgets();
        m_ds.InTest(true);
        test();
        m_ds.InTest(false);
        while (isTest() && isEnabled()) {
          m_ds.waitForData();
        }
        LiveWindow.setEnabled(false);
        Shuffleboard.disableActuatorWidgets();
      } else {
        m_ds.InOperatorControl(true);
        teleop();
        m_ds.InOperatorControl(false);
        while (isOperatorControl() && !isDisabled()) {
          m_ds.waitForData();
        }
      }
    }
  }

  @Override
  public void endCompetition() {
    m_exit = true;
  }
}
