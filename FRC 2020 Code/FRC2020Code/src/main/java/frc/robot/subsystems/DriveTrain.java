/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;




public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */

  static final double kToleranceDegrees = 2.0f;

   private TalonSRX DriveLF;
   private TalonSRX DriveRF;
   private TalonSRX DriveLB;
   private TalonSRX DriveRB;

   private static final int joysick = 1;
  
   private static final int kdTimeoutMs = 30;                      //set to zero to skip error checking
   private static final double kdDeadBand = 0.02; 

  public DriveTrain() {
    final TalonSRX DriveLF = new TalonSRX(1);
    DriveLF.configFactoryDefault();
    DriveLF.setInverted(true);
    DriveLF.setSensorPhase(true);
    DriveLF.configNeutralDeadband(kdDeadBand);
    DriveLF.setNeutralMode(NeutralMode.Coast);

    DriveLF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kdTimeoutMs);
    DriveLF.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kdTimeoutMs);

    DriveLF.configNominalOutputForward(0, kdTimeoutMs);
    DriveLF.configNominalOutputReverse(0, kdTimeoutMs);
    DriveLF.configPeakOutputForward(1, kdTimeoutMs);
    DriveLF.configPeakOutputReverse(-1, kdTimeoutMs);

    final TalonSRX DriveRF = new TalonSRX(2);
    DriveRF.configFactoryDefault();
    DriveRF.setInverted(true);
    DriveRF.setSensorPhase(true);
    DriveRF.configNeutralDeadband(kdDeadBand);
    // DriveRF.setNeutralMode(NeutralMode.Coast);

    // DriveRF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
    // 0, kdTimeoutMS);
    // DriveRF.setsStatusFramePeriod(StatusFrameEnhanced.Status_13_Bade_Base_PIDF0,
    // 10, kdTimeoutMs);

    // DriveRF.configNominalOutputFoward(0, kdTimeoutMs);
    DriveRF.configNominalOutputReverse(0, kdTimeoutMs);
    DriveRF.configPeakOutputForward(1, kdTimeoutMs);
    DriveRF.configPeakOutputReverse(-1, kdTimeoutMs);

    final TalonSRX DriveRB = new TalonSRX(3);
    DriveRB.configFactoryDefault();
    DriveRB.setInverted(true);
    DriveRB.setSensorPhase(true);
    DriveRB.configNeutralDeadband(kdDeadBand);
    DriveRB.setNeutralMode(NeutralMode.Coast);

    DriveRB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kdTimeoutMs);
    DriveRB.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kdTimeoutMs);

    DriveRB.configNominalOutputForward(0, kdTimeoutMs);
    DriveRB.configNominalOutputReverse(0, kdTimeoutMs);
    DriveRB.configPeakOutputForward(1, kdTimeoutMs);
    DriveRB.configPeakOutputReverse(-1, kdTimeoutMs);

    final TalonSRX DriveLB = new TalonSRX(4);
    DriveLB.configFactoryDefault();
    DriveLB.setInverted(true);
    DriveLB.setSensorPhase(true);
    DriveLB.configNeutralDeadband(kdDeadBand);
    DriveLB.setNeutralMode(NeutralMode.Coast);

    DriveLB.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, kdTimeoutMs);
    DriveLB.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,
    10, kdTimeoutMs);

    DriveLB.configNominalOutputForward(0, kdTimeoutMs);
    DriveLB.configNominalOutputReverse(0, kdTimeoutMs);
    DriveLB.configPeakOutputForward(1, kdTimeoutMs);
    DriveLB.configPeakOutputReverse(-1, kdTimeoutMs);



  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void DriveWithJoy() { 
        
        
    double Jy=-1*RobotContainer.xbox.getY(Hand.kLeft) * 1;
    double Jx=RobotContainer.xbox.getX(Hand.kLeft)*1;
    double Jz=RobotContainer.xbox.getX(Hand.kRight)*1;


    
    double r = Math.hypot(Jx, Jy);
    double robotAngle = Math.atan2(Jy, Jx) - Math.PI / 4;
    double rightX = Jz;
    var LF = r * Math.cos(robotAngle) + rightX;
    double RF = r * Math.sin(robotAngle) - rightX;
    double LR = r * Math.sin(robotAngle) + rightX;
    double RR = r * Math.cos(robotAngle) - rightX;
    System.out.println("LF:" + LF + "  RF:" + RF );
    System.out.println("LR:" + LR + "  RR:" + RR );
    System.out.println(" ");

    //SmartDashboard.putNumber("LF", LF);
    //SmartDashboard.putNumber("RF", RF);
    //SmartDashboard.putNumber("LR", LR);
    //SmartDashboard.putNumber("RR", RR);

    //leftFront.setPower(v1); old old from origianl math code example
    //rightFront.setPower(v2);
    //leftRear.setPower(v3)
    //rightRear.setPower(v4);
    
    
   
   // DriveRF.set(RF);         
    //DriveLR.set(LR);
    //DriveRR.set(RR);

    DriveLF.set(ControlMode.PercentOutput, LF, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive     
    DriveLB.set(ControlMode.PercentOutput, LR, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive
   DriveRF.set(ControlMode.PercentOutput, RF, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive
   DriveRB.set(ControlMode.PercentOutput, RR, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive


  }
   public void ApplyMotorPower(double L, double R) {                    // Tankdrive Percent output
        // This modifies power applied to the motors directly.

        // Speed reduction
        // Original design:  Both triggers is reduction2, one trigger is reduction 1.
        // This allows either trigger to be used for reduction 1 and both results in reduction 2.
       /* if(RobotContainer.rightStick.getTrigger()&&Robot.oi.leftStick.getTrigger()){
            L=L*Constants.DRIVE_POWER_REDUCTION2;
            R=R*Constants.DRIVE_POWER_REDUCTION2;
        } else if(Robot.oi.rightStick.getTrigger()||Robot.oi.leftStick.getTrigger()){
            L=L*Constants.DRIVE_POWER_REDUCTION;
            R=R*Constants.DRIVE_POWER_REDUCTION;
        }
*/

        //Apply the power to the drive.
        DriveLF.set(ControlMode.PercentOutput, L, DemandType.ArbitraryFeedForward, 0);  // this was oringaly used for arcade drive
        DriveRF.set(ControlMode.PercentOutput, R, DemandType.ArbitraryFeedForward, 0);  // With the zero at the end I don't think it does anything different   

    }
}
