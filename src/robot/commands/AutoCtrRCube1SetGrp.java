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

import edu.wpi.first.wpilibj.command.CommandGroup;
import robot.subsystems.*;

/**
 *
 */
public class AutoCtrRCube1SetGrp extends CommandGroup {


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
    public AutoCtrRCube1SetGrp() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=PARAMETERS
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=COMMAND_DECLARATIONS
 
    	// ******************************************************************************************
    	// 				 					Drive to Right Switch 							 		* 
    	// ******************************************************************************************   	 
   	//					armToPosCmd(position,  speed, mode,   timeOut
   	addParallel(new 	armToPosCmd("Switch",   0.0,   3.0,    3.0));
   	   
   	//				  DriveArcTurnCmd( leftPwr, rightPwr, dist,   hdg, mode, TO, Brake,  PIDMode)
   	addSequential(new DriveArcTurnCmd(   0.85,      0.25,     0,   48,   1,   5, false,   false));    	
   	addSequential(new DriveArcTurnCmd(   0.25,      0.85,     0,   5 ,   1,   5, false,   false));
   	
   	// 			      				 	(tgtist,    Pwr,     Hdg,   Mode,   TO,   Brake, LimitOveride)    	
   	addSequential(new DriveFwd2Cmd		(25.0,      0.7,    0.0,     1,    3.0,  false,  false));
   	addSequential(new DriveBrakeCmd		(3.0));   
   	
   	
 	
	// ******************************************************************************************
	// 			 Drive to Switch and Place first cube on Switch and setup for second cude		* 
	// 				-------------- USED IN CLACKAMAS COMEPTITION ----------------------			*
	// ******************************************************************************************  	
	/*
	 ***** Original Code ****
	addParallel(new LiftToSwitchCmdGrp	());
	    	
	// 			      				 		(tgtDist,    Mode,   TO,   Hdg)
	addSequential(new DriveFwdPidCmd		(  23.0,      1,    5.0,   0.0));

	//										( turnAngle,  mode, TO )
	addSequential(new DrivePointTurnPidCmd	(  32,      0,    3.0));
	
	// 			      				 		(tgtDist,    Mode,   TO,   Hdg)
	addSequential(new DriveFwdPidCmd		(  90.0,      1,    5.0,   32.0));
	
	//										( turnAngle,  mode, TO )
	addSequential(new DrivePointTurnPidCmd	(  0,      0,    3.0));    	
	
	addSequential(new IntakeEjectCmd		());
	

	// ------ Setup for second cube ------
  	// 			      				 		(tgtDist,    Mode,   TO,   Hdg)
	//addSequential(new DriveFwdPidCmd		(  -21.0,      1,    5.0,   0.0));
	
	//addParallel(new LiftToBottomCmdGrp	());
	
	//										( turnAngle,  mode, TO )
	addSequential(new DrivePointTurnPidCmd	(  -55,      0,    3.0));
	
	 */
    } 
}
