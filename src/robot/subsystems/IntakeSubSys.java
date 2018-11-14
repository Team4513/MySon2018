// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot.subsystems;

import robot.Robot;
import robot.RobotMap;
import robot.commands.*;
import robot.utils.Rmath;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class IntakeSubSys extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController intakeMtr = RobotMap.intakeSubSysIntakeMtr;
    private final SpeedController intakeLeftMtr = RobotMap.intakeSubSysIntakeLeftMtr;
    private final SpeedController intakeRightMtr = RobotMap.intakeSubSysIntakeRightMtr;
    private final AnalogInput intakeSensor = RobotMap.intakeSubSysIntakeSensor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public final boolean	CONTACTSWITCHPRESSED = true;
    public final boolean	CONTACTSWITCHNOTPRESSED = false;
    public final double		RETRACT_PWR = +0.9;
    public final double		EJECT_PWR = -0.75;
    public final double		HOLD_PWR = +0.20;
    public final double		CUBE_CAPTURED_VAL = 1.5;
    
    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    	update_Smartdashboard();						// displays limit switches states
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	//							*********Begin Intake Methods*************

	
    public void retract(){
   		intakeLeftMtr.set(RETRACT_PWR);
   		intakeRightMtr.set(-RETRACT_PWR);
    }
    
    public void retract(double pwr){
        if (pwr < 0) pwr *= -1;     // Power should be positive to retract     
   		intakeLeftMtr.set(RETRACT_PWR);
   		intakeRightMtr.set(-RETRACT_PWR);
    }

    public void retractStop(){     
		intakeLeftMtr.set(0);
		intakeRightMtr.set(0);
    }
    
    public void eject(){
		intakeLeftMtr.set(EJECT_PWR);
		intakeRightMtr.set(-EJECT_PWR);
    }
    
    public void eject(double pwr){
        if (pwr > 0) pwr *= -1;     // Power should be negative to eject 
		intakeLeftMtr.set(pwr);
		intakeRightMtr.set(-pwr);
    }

    
    public void drive(double pwr){
        if (pwr > 0) pwr *= -1;     // Power should be negative to eject 
		intakeLeftMtr.set(pwr);
		intakeRightMtr.set(-pwr);
    }
    
    public void hold(){
		intakeLeftMtr.set(HOLD_PWR);
		intakeRightMtr.set(-HOLD_PWR);
    }
    
    public boolean haveCube() {
    	if (intakeSensor.getAverageVoltage() >= CUBE_CAPTURED_VAL)
    		return true;
    	else
    		return false;
    }
    
    void update_Smartdashboard(){
    	if (haveCube())
    		SmartDashboard.putString("Intake", "HAS Cube");
    	else
    		SmartDashboard.putString("Intake", "NO Cube");
    	
    	SmartDashboard.putNumber("Sensor Value", Rmath.mRound(intakeSensor.getAverageVoltage(),2));
    }

}

