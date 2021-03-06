// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package robot.commands;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

/**
 *
 */
public class IntakeEjectByJoyCmd extends Command {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

	static final double kEjectSpeed = -1;
	static String line;
	private double joyThrottle, throttle, motorSpeed;

	Joystick joyCo,joyDrive;
	
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public IntakeEjectByJoyCmd() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.intakeSubSys);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    	setTimeout(0.25);
    	line = "Intake Eject By Joy Command Called!" ;
    	System.out.println(line) ;
    	Robot.logger.appendLog(line);
    	joyCo= Robot.oi.coPilotJoystick;
    	joyDrive=Robot.oi.driverJoystick;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    	
		//Robot.intakeSubSys.eject();
		calcEjectMtrSpd();		// This updates motorSpeed
		Robot.intakeSubSys.drive(motorSpeed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
    	if(isTimedOut()){
    		return true;
    	}
    	
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    	Robot.intakeSubSys.retractStop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    	end();
    }
    
    
    void calcEjectMtrSpd(){
    	joyThrottle = joyCo.getThrottle();			//get throttle value
    	throttle = (((joyThrottle * -1) +1 ) / 2);				//adjust throttle value from -1 to 1 => 0 to 1
    	motorSpeed = throttle;
    }
}
