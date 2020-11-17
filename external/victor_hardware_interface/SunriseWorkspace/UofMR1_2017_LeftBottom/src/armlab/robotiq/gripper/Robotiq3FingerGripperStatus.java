package armlab.robotiq.gripper;

import armlab.lcm.msgs.*;
import armlab.utils.Utils;

public class Robotiq3FingerGripperStatus
{
	// Gripper status register
	private final static int gACT_RESET = 0x0;
	private final static int gACT_ACTIVATION = 0x1;
	private final static int gMOD_BASIC = 0x0;
	private final static int gMOD_PINCH = 0x1;
	private final static int gMOD_WIDE = 0x2;
	private final static int gMOD_SCISSOR = 0x3;
	private final static int gGTO_STOPPED = 0x0;
	private final static int gGTO_GOTO = 0x1;
	private final static int gIMC_RESET_AUTO_RELEASE = 0x0;
	private final static int gIMC_ACTIVATION_IN_PROGRESS = 0x1;
	private final static int gIMC_MODE_CHANGE_IN_PROGRESS = 0x2;
	private final static int gIMC_ACTIVATION_MODE_CHANGE_COMPLETE = 0x3;
	private final static int gSTA_GRIPPER_IN_MOTION = 0x0;
	private final static int gSTA_GRIPPER_ONE_OR_TWO_STOPPED_EARLY = 0x1;
	private final static int gSTA_GRIPPER_ALL_STOPPED_EARLY = 0x2;
	private final static int gSTA_GRIPPER_ALL_AT_REQUESTED = 0x3;
	// Object status register
	private final static int gDTX_IN_MOTION = 0x0;
	private final static int gDTX_STOPPED_OPENING = 0x1;
	private final static int gDTX_STOPPED_CLOSING = 0x2;
	private final static int gDTX_AT_REQUESTED = 0x3;
	// Gripper fault register
	private final static int gFLT_NO_FAULT = 0x0;
	private final static int gFLT_ACTION_DELAYED_ACTIVATION_NEEDED = 0x5;
	private final static int gFLT_ACTION_DELAYED_MODE_CHANGE_NEEDED = 0x6;
	private final static int gFLT_ACTIVATION_MUST_BE_SET = 0x7;
	private final static int gFLT_COMM_CHIP_NOT_READY = 0x9;
	private final static int gFLT_CHANGING_MODE_FAULT_MINOR_SCISSOR_INTERFERENCE = 0xA;
	private final static int gFLT_AUTORELEASE_IN_PROGRESS = 0xB;
	private final static int gFLT_ACTIVATION_FAULT = 0xD;
	private final static int gFLT_CHANGING_MODE_FAULT_MAJOR_SCISSOR_INTERFERENCE = 0xE;
	private final static int gFLT_AUTORELEASE_COMPLETE = 0xF;
	
	public enum InitializationStatus { GRIPPER_RESET, GRIPPER_ACTIVATION };
	public enum GripperModeStatus { MODE_BASIC, MODE_PINCH, MODE_WIDE, MODE_SCISSOR };
	public enum GripperActionStatus { GRIPPER_STOPPED_OR_BUSY, GRIPPER_GOTO };
	public enum GripperSystemStatus { GRIPPER_RESET_OR_AUTO_RELEASE, GRIPPER_ACTIVATION_IN_PROGRESS, GRIPPER_MODE_CHANGE_IN_PROGESS, GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE };
	public enum GripperMotionStatus { GRIPPER_STOPPED_UNKNOWN, GRIPPER_IN_MOTION, GRIPPER_ONE_OR_TWO_STOPPED_EARLY, GRIPPER_ALL_STOPPED_EARLY, GRIPPER_ALL_AT_REQUESTED }; 
	public enum ObjectStatus { STOPPED, IN_MOTION, CONTACT_OPENING, CONTACT_CLOSING, AT_REQUESTED };
	public enum GripperFaultStatus { NO_FAULTS, PRIORITY_ACTIVATION_MUST_BE_SET, PRIORITY_MODE_CHANGE_NEEDED, PRIORITY_NEEDS_ACTIVATION,
									 MINOR_COMM_CHIP_NOT_READY, MINOR_CHANGING_MODE_FAULT, MINOR_AUTO_RELEASE_IN_PROGRESS,
									 MAJOR_ACTIVATION_FAULT, MAJOR_CHANGING_MODE_FAULT, MAJOR_AUTO_RELEASE_COMPLETE };
									 
	private InitializationStatus initialization_status_;
	private GripperActionStatus gripper_action_status_;
	private GripperSystemStatus gripper_system_status_;
	private GripperMotionStatus gripper_motion_status_;
	private GripperFaultStatus gripper_fault_status_;
	private ObjectStatus finger_a_object_status_;
	private ObjectStatus finger_b_object_status_;
	private ObjectStatus finger_c_object_status_;
	private ObjectStatus scissor_object_status_;
	private double finger_a_position_request_;
	private double finger_a_position_;
	private double finger_a_current_;
	private double finger_b_position_request_;
	private double finger_b_position_;
	private double finger_b_current_;
	private double finger_c_position_request_;
	private double finger_c_position_;
	private double finger_c_current_;
	private double scissor_position_request_;
	private double scissor_position_;
	private double scissor_current_;
	
	public static robotiq_3finger_status ConvertToLCMMessage(Robotiq3FingerGripperStatus status)
	{
		final double now = Utils.getUTCTimeAsDouble();
		
		robotiq_3finger_status lcm_status = new robotiq_3finger_status();
		lcm_status.timestamp = now;
		
		lcm_status.finger_a_status = new robotiq_3finger_actuator_status();
		lcm_status.finger_a_status.position_request = status.finger_a_position_request_;
		lcm_status.finger_a_status.position = status.finger_a_position_;
		lcm_status.finger_a_status.current = status.finger_a_current_;
		lcm_status.finger_a_status.timestamp = now;
		
		lcm_status.finger_b_status = new robotiq_3finger_actuator_status();
		lcm_status.finger_b_status.position_request = status.finger_b_position_request_;
		lcm_status.finger_b_status.position = status.finger_b_position_;
		lcm_status.finger_b_status.current = status.finger_b_current_;
		lcm_status.finger_b_status.timestamp = now;
		
		lcm_status.finger_c_status = new robotiq_3finger_actuator_status();
		lcm_status.finger_c_status.position_request = status.finger_c_position_request_;
		lcm_status.finger_c_status.position = status.finger_c_position_;
		lcm_status.finger_c_status.current = status.finger_c_current_;
		lcm_status.finger_c_status.timestamp = now;
		
		lcm_status.scissor_status = new robotiq_3finger_actuator_status();
		lcm_status.scissor_status.position_request = status.scissor_position_request_;
		lcm_status.scissor_status.position = status.scissor_position_;
		lcm_status.scissor_status.current = status.scissor_current_;
		lcm_status.scissor_status.timestamp = now;
		
		lcm_status.finger_a_object_status = new robotiq_3finger_object_status();
		lcm_status.finger_a_object_status.status = (byte) status.finger_a_object_status_.ordinal();
		lcm_status.finger_a_object_status.timestamp = now;
		
		lcm_status.finger_b_object_status = new robotiq_3finger_object_status();
		lcm_status.finger_b_object_status.status = (byte) status.finger_b_object_status_.ordinal();
		lcm_status.finger_b_object_status.timestamp = now;
		
		lcm_status.finger_c_object_status = new robotiq_3finger_object_status();
		lcm_status.finger_c_object_status.status = (byte) status.finger_c_object_status_.ordinal();
		lcm_status.finger_c_object_status.timestamp = now;
		
		lcm_status.scissor_object_status = new robotiq_3finger_object_status();
		lcm_status.scissor_object_status.status = (byte) status.scissor_object_status_.ordinal();
		lcm_status.scissor_object_status.timestamp = now;
		
		return lcm_status;
	}

	public static void PopulateLCMStatusMessage(robotiq_3finger_status lcm_status,
										  double now,
										  int gripper_status_register,
										  int object_status_register,
										  int gripper_fault_register,
										  int finger_a_position_request,
										  int finger_a_position,
										  int finger_a_current,
										  int finger_b_position_request,
										  int finger_b_position,
										  int finger_b_current,
										  int finger_c_position_request,
										  int finger_c_position,
										  int finger_c_current,
										  int scissor_position_request,
										  int scissor_position,
										  int scissor_current)
	{
		DecodeGripperRegistersLCM(lcm_status, gripper_status_register, object_status_register, gripper_fault_register);
		lcm_status.timestamp = now;
		lcm_status.finger_a_object_status.timestamp = now;
		lcm_status.finger_b_object_status.timestamp = now;
		lcm_status.finger_c_object_status.timestamp = now;
		lcm_status.scissor_object_status.timestamp = now;
		
		lcm_status.finger_a_status.position_request = ConvertFromStatusValue(finger_a_position_request);
		lcm_status.finger_a_status.position = ConvertFromStatusValue(finger_a_position);
		lcm_status.finger_a_status.current = ConvertFromStatusValue(finger_a_current);
		lcm_status.finger_a_status.timestamp = now;
		
		lcm_status.finger_b_status.position_request = ConvertFromStatusValue(finger_b_position_request);
		lcm_status.finger_b_status.position = ConvertFromStatusValue(finger_b_position);
		lcm_status.finger_b_status.current = ConvertFromStatusValue(finger_b_current);
		lcm_status.finger_b_status.timestamp = now;
		
		lcm_status.finger_c_status.position_request = ConvertFromStatusValue(finger_c_position_request);
		lcm_status.finger_c_status.position = ConvertFromStatusValue(finger_c_position);
		lcm_status.finger_c_status.current = ConvertFromStatusValue(finger_c_current);
		lcm_status.finger_c_status.timestamp = now;
		
		lcm_status.scissor_status.position_request = ConvertFromStatusValue(scissor_position_request);
		lcm_status.scissor_status.position = ConvertFromStatusValue(scissor_position);
		lcm_status.scissor_status.current = ConvertFromStatusValue(scissor_current);
		lcm_status.scissor_status.timestamp = now;
	}
	
	public static InitializationStatus DecodeGripperInitializationStatus(int gripper_status_register)
	{
		final int initialization_status_value = gripper_status_register & 0x1;
		if (initialization_status_value == gACT_RESET)
		{
			return InitializationStatus.GRIPPER_RESET;
		}
		else
		{
			assert(initialization_status_value == gACT_ACTIVATION);
			return InitializationStatus.GRIPPER_ACTIVATION;
		}
	}
	
	public static GripperModeStatus DecodeGripperModeStatus(int gripper_status_register)
	{
		final int mode_status_value = (gripper_status_register >> 1) & 0x3;
		if (mode_status_value == gMOD_BASIC)
		{
			return GripperModeStatus.MODE_BASIC;
		}
		else if (mode_status_value == gMOD_PINCH)
		{
			return GripperModeStatus.MODE_PINCH;
		}
		else if (mode_status_value == gMOD_WIDE)
		{
			return GripperModeStatus.MODE_WIDE;
		}
		else
		{
			assert(mode_status_value == gMOD_SCISSOR);
			return GripperModeStatus.MODE_SCISSOR;
		}
	}
	
	public static GripperActionStatus DecodeGripperActionStatus(int gripper_status_register)
	{
		final int action_status_value = (gripper_status_register >> 3) & 0x1;
		if (action_status_value == gGTO_STOPPED)
		{
			return GripperActionStatus.GRIPPER_STOPPED_OR_BUSY;
		}
		else
		{
			assert(action_status_value == gGTO_GOTO);
			return GripperActionStatus.GRIPPER_GOTO;
		}
	}
	
	public static GripperSystemStatus DecodeGripperSystemStatus(int gripper_status_register)
	{
		final int system_status_value = (gripper_status_register >> 4) & 0x3;
		if (system_status_value == gIMC_RESET_AUTO_RELEASE)
		{
			return GripperSystemStatus.GRIPPER_RESET_OR_AUTO_RELEASE;
		}
		else if (system_status_value == gIMC_ACTIVATION_IN_PROGRESS)
		{
			return GripperSystemStatus.GRIPPER_ACTIVATION_IN_PROGRESS;
		}
		else if (system_status_value == gIMC_MODE_CHANGE_IN_PROGRESS)
		{
			return GripperSystemStatus.GRIPPER_MODE_CHANGE_IN_PROGESS;
		}
		else
		{
			assert(system_status_value == gIMC_ACTIVATION_MODE_CHANGE_COMPLETE);
			return GripperSystemStatus.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE;
		}
	}
	
	public static GripperMotionStatus DecodeGripperMotionStatus(int gripper_status_register)
	{
		final int motion_status_value = (gripper_status_register >> 6) & 0x3;
		if (motion_status_value == gSTA_GRIPPER_IN_MOTION)
		{
			final GripperActionStatus gripper_action_status = DecodeGripperActionStatus(gripper_status_register);
			if (gripper_action_status == GripperActionStatus.GRIPPER_GOTO)
			{
				return GripperMotionStatus.GRIPPER_IN_MOTION;
			}
			else
			{
				return GripperMotionStatus.GRIPPER_STOPPED_UNKNOWN;
			}
		}
		else if (motion_status_value == gSTA_GRIPPER_ONE_OR_TWO_STOPPED_EARLY)
		{
			return GripperMotionStatus.GRIPPER_ONE_OR_TWO_STOPPED_EARLY;
		}
		else if (motion_status_value == gSTA_GRIPPER_ALL_STOPPED_EARLY)
		{
			return GripperMotionStatus.GRIPPER_ALL_STOPPED_EARLY;
		}
		else
		{
			assert(motion_status_value == gSTA_GRIPPER_ALL_AT_REQUESTED);
			return GripperMotionStatus.GRIPPER_ALL_AT_REQUESTED;
		}
	}
	
	private void DecodeGripperStatusRegister(int gripper_status_register)
	{
		initialization_status_ = DecodeGripperInitializationStatus(gripper_status_register);
		gripper_action_status_ = DecodeGripperActionStatus(gripper_status_register);
		gripper_system_status_ = DecodeGripperSystemStatus(gripper_status_register);
		gripper_motion_status_ = DecodeGripperMotionStatus(gripper_status_register);
	}
	
	private static void DecodeGripperStatusRegisterLCM(robotiq_3finger_status lcm_status, int gripper_status_register)
	{
		lcm_status.initialization_status = (byte) DecodeGripperInitializationStatus(gripper_status_register).ordinal();
		lcm_status.gripper_action_status = (byte) DecodeGripperActionStatus(gripper_status_register).ordinal();
		lcm_status.gripper_system_status = (byte) DecodeGripperSystemStatus(gripper_status_register).ordinal();
		lcm_status.gripper_motion_status = (byte) DecodeGripperMotionStatus(gripper_status_register).ordinal();
	}
	
	private static ObjectStatus DecodeObjectStatusRegister(int object_status_value, int gripper_status_register)
	{
		if (object_status_value == gDTX_IN_MOTION)
		{
			final GripperActionStatus gripper_action_status = DecodeGripperActionStatus(gripper_status_register);
			if (gripper_action_status == GripperActionStatus.GRIPPER_GOTO)
			{
				return ObjectStatus.IN_MOTION;
			}
			else
			{
				return ObjectStatus.STOPPED;
			}
		}
		else if (object_status_value == gDTX_STOPPED_OPENING)
		{
			return ObjectStatus.CONTACT_OPENING;
		}
		else if (object_status_value == gDTX_STOPPED_CLOSING)
		{
			return ObjectStatus.CONTACT_CLOSING;
		}
		else
		{
			assert(object_status_value == gDTX_AT_REQUESTED);
			return ObjectStatus.AT_REQUESTED;
		}
	}
	
	private void DecodeObjectStatusRegisters(int object_status_register, int gripper_status_register)
	{
		final int finger_a_object_status = object_status_register & 0x3;
		final int finger_b_object_status = (object_status_register >> 2) & 0x3;
		final int finger_c_object_status = (object_status_register >> 4) & 0x3;
		final int scissor_object_status = (object_status_register >> 6) & 0x3;
		finger_a_object_status_ = DecodeObjectStatusRegister(finger_a_object_status, gripper_status_register);
		finger_b_object_status_ = DecodeObjectStatusRegister(finger_b_object_status, gripper_status_register);
		finger_c_object_status_ = DecodeObjectStatusRegister(finger_c_object_status, gripper_status_register);
		scissor_object_status_ = DecodeObjectStatusRegister(scissor_object_status, gripper_status_register);
	}
	
	private static void DecodeObjectStatusRegistersLCM(robotiq_3finger_status lcm_status, int object_status_register, int gripper_status_register)
	{
		final int finger_a_object_status = object_status_register & 0x3;
		final int finger_b_object_status = (object_status_register >> 2) & 0x3;
		final int finger_c_object_status = (object_status_register >> 4) & 0x3;
		final int scissor_object_status = (object_status_register >> 6) & 0x3;
		lcm_status.finger_a_object_status.status = (byte) DecodeObjectStatusRegister(finger_a_object_status, gripper_status_register).ordinal();
		lcm_status.finger_a_object_status.status = (byte) DecodeObjectStatusRegister(finger_b_object_status, gripper_status_register).ordinal();
		lcm_status.finger_a_object_status.status = (byte) DecodeObjectStatusRegister(finger_c_object_status, gripper_status_register).ordinal();
		lcm_status.finger_a_object_status.status = (byte) DecodeObjectStatusRegister(scissor_object_status, gripper_status_register).ordinal();
	}
	
	private void DecodeGripperFaultRegister(int gripper_fault_register)
	{
		final int fault_status_value = gripper_fault_register & 0xF;
		if (fault_status_value == gFLT_NO_FAULT)
		{
			gripper_fault_status_ = GripperFaultStatus.NO_FAULTS;
		}
		else if (fault_status_value == gFLT_ACTION_DELAYED_ACTIVATION_NEEDED)
		{
			gripper_fault_status_ = GripperFaultStatus.PRIORITY_NEEDS_ACTIVATION;
		}
		else if (fault_status_value == gFLT_ACTION_DELAYED_MODE_CHANGE_NEEDED)
		{
			gripper_fault_status_ = GripperFaultStatus.PRIORITY_MODE_CHANGE_NEEDED;
		}
		else if (fault_status_value == gFLT_ACTIVATION_MUST_BE_SET)
		{
			gripper_fault_status_ = GripperFaultStatus.PRIORITY_ACTIVATION_MUST_BE_SET;
		}
		else if (fault_status_value == gFLT_COMM_CHIP_NOT_READY)
		{
			gripper_fault_status_ = GripperFaultStatus.MINOR_COMM_CHIP_NOT_READY;
		}
		else if (fault_status_value == gFLT_CHANGING_MODE_FAULT_MINOR_SCISSOR_INTERFERENCE)
		{
			gripper_fault_status_ = GripperFaultStatus.MINOR_CHANGING_MODE_FAULT;
		}
		else if (fault_status_value == gFLT_AUTORELEASE_IN_PROGRESS)
		{
			gripper_fault_status_ = GripperFaultStatus.MINOR_AUTO_RELEASE_IN_PROGRESS;
		}
		else if (fault_status_value == gFLT_ACTIVATION_FAULT)
		{
			gripper_fault_status_ = GripperFaultStatus.MAJOR_ACTIVATION_FAULT;
		}
		else if (fault_status_value == gFLT_CHANGING_MODE_FAULT_MAJOR_SCISSOR_INTERFERENCE)
		{
			gripper_fault_status_ = GripperFaultStatus.MAJOR_CHANGING_MODE_FAULT;
		}
		else if (fault_status_value == gFLT_AUTORELEASE_COMPLETE)
		{
			gripper_fault_status_ = GripperFaultStatus.MAJOR_AUTO_RELEASE_COMPLETE;
		}
	}
	
	private static void DecodeGripperFaultRegisterLCM(robotiq_3finger_status lcm_status, int gripper_fault_register)
	{
		final int fault_status_value = gripper_fault_register & 0xF;
		if (fault_status_value == gFLT_NO_FAULT)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.NO_FAULTS;
		}
		else if (fault_status_value == gFLT_ACTION_DELAYED_ACTIVATION_NEEDED)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.PRIORITY_NEEDS_ACTIVATION;
		}
		else if (fault_status_value == gFLT_ACTION_DELAYED_MODE_CHANGE_NEEDED)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.PRIORITY_MODE_CHANGE_NEEDED;
		}
		else if (fault_status_value == gFLT_ACTIVATION_MUST_BE_SET)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.PRIORITY_ACTIVATION_MUST_BE_SET;
		}
		else if (fault_status_value == gFLT_COMM_CHIP_NOT_READY)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.MINOR_COMM_CHIP_NOT_READY;
		}
		else if (fault_status_value == gFLT_CHANGING_MODE_FAULT_MINOR_SCISSOR_INTERFERENCE)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.MINOR_CHANGING_MODE_FAULT;
		}
		else if (fault_status_value == gFLT_AUTORELEASE_IN_PROGRESS)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.MINOR_AUTO_RELEASE_IN_PROGRESS;
		}
		else if (fault_status_value == gFLT_ACTIVATION_FAULT)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.MAJOR_ACTIVATION_FAULT;
		}
		else if (fault_status_value == gFLT_CHANGING_MODE_FAULT_MAJOR_SCISSOR_INTERFERENCE)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.MAJOR_CHANGING_MODE_FAULT;
		}
		else if (fault_status_value == gFLT_AUTORELEASE_COMPLETE)
		{
			lcm_status.gripper_fault_status = robotiq_3finger_status.MAJOR_AUTO_RELEASE_COMPLETE;
		}
	}
	
	public static double ConvertFromStatusValue(int status_value)
	{
		double value = (double)status_value / 255.0;
		return value;
	}
	
	private void DecodeGripperRegisters(int gripper_status_register, int object_status_register, int gripper_fault_register)
	{
		DecodeGripperStatusRegister(gripper_status_register);
		DecodeObjectStatusRegisters(object_status_register, gripper_status_register);
		DecodeGripperFaultRegister(gripper_fault_register);
	}
	
	private static void DecodeGripperRegistersLCM(robotiq_3finger_status lcm_status, int gripper_status_register, int object_status_register, int gripper_fault_register)
	{
		DecodeGripperStatusRegisterLCM(lcm_status, gripper_status_register);
		DecodeObjectStatusRegistersLCM(lcm_status, object_status_register, gripper_status_register);
		DecodeGripperFaultRegisterLCM(lcm_status, gripper_fault_register);
	}
	
	public Robotiq3FingerGripperStatus(int gripper_status_register,
									   int object_status_register,
									   int gripper_fault_register,
									   int finger_a_position_request,
									   int finger_a_position,
									   int finger_a_current,
									   int finger_b_position_request,
									   int finger_b_position,
									   int finger_b_current,
									   int finger_c_position_request,
									   int finger_c_position,
									   int finger_c_current,
									   int scissor_position_request,
									   int scissor_position,
									   int scissor_current)
	{
		DecodeGripperRegisters(gripper_status_register, object_status_register, gripper_fault_register);
		finger_a_position_request_ = ConvertFromStatusValue(finger_a_position_request);
		finger_a_position_ = ConvertFromStatusValue(finger_a_position);
		finger_a_current_ = ConvertFromStatusValue(finger_a_current);
		finger_b_position_request_ = ConvertFromStatusValue(finger_b_position_request);
		finger_b_position_ = ConvertFromStatusValue(finger_b_position);
		finger_b_current_ = ConvertFromStatusValue(finger_b_current);
		finger_a_position_request_ = ConvertFromStatusValue(finger_b_position_request);
		finger_a_position_ = ConvertFromStatusValue(finger_b_position);
		finger_a_current_ = ConvertFromStatusValue(finger_b_current);
		scissor_position_request_ = ConvertFromStatusValue(scissor_position_request);
		scissor_position_ = ConvertFromStatusValue(scissor_position);
		scissor_current_ = ConvertFromStatusValue(scissor_current);
	}
	
	public boolean IsActivated()
	{
		if (initialization_status_ == InitializationStatus.GRIPPER_ACTIVATION && gripper_system_status_ == GripperSystemStatus.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean IsStopped()
	{
		System.out.println("Gripper status: " + PrintGripperMotionStatus(gripper_motion_status_)
							+ ", Finger status: " + PrintObjectStatus(finger_a_object_status_)
									  			  + ", " + PrintObjectStatus(finger_b_object_status_)
									  			  + ", " + PrintObjectStatus(finger_c_object_status_)
									  			  + ", " + PrintObjectStatus(scissor_object_status_));
		if (finger_a_object_status_ == ObjectStatus.IN_MOTION)
		{
			return false;
		}
		if (finger_b_object_status_ == ObjectStatus.IN_MOTION)
		{
			return false;
		}
		if (finger_c_object_status_ == ObjectStatus.IN_MOTION)
		{
			return false;
		}
		if (scissor_object_status_ == ObjectStatus.IN_MOTION)
		{
			return false;
		}
		return true;
	}
	
	public String PrintObjectStatus(ObjectStatus object_status)
	{
		if (object_status == ObjectStatus.IN_MOTION)
		{
			return "In Motion";
		}
		else if (object_status == ObjectStatus.CONTACT_CLOSING)
		{
			return "Contact Closing";
		}
		else if (object_status == ObjectStatus.CONTACT_OPENING)
		{
			return "Contact Opening";
		}
		else if (object_status == ObjectStatus.AT_REQUESTED)
		{
			return "At Requested";
		}
		else
		{
			assert(object_status == ObjectStatus.STOPPED);
			return "Stopped";
		}
	}
	
	public String PrintGripperMotionStatus(GripperMotionStatus gripper_motion_status)
	{
		if (gripper_motion_status == GripperMotionStatus.GRIPPER_IN_MOTION)
		{
			return "Gripper In Motion";
		}
		else if (gripper_motion_status == GripperMotionStatus.GRIPPER_ONE_OR_TWO_STOPPED_EARLY)
		{
			return "Gripper One Or Two Stopped Early";
		}
		else if (gripper_motion_status == GripperMotionStatus.GRIPPER_ALL_STOPPED_EARLY)
		{
			return "Gripper All Stopped Early";
		}
		else if (gripper_motion_status == GripperMotionStatus.GRIPPER_ALL_AT_REQUESTED)
		{
			return "Gripper All At Requested";
		}
		else
		{
			assert(gripper_motion_status == GripperMotionStatus.GRIPPER_STOPPED_UNKNOWN);
			return "Gripper Stopped/Unknown";
		}
	}
	
	public boolean InCollision()
	{
		if (finger_a_object_status_ == ObjectStatus.CONTACT_CLOSING || finger_a_object_status_ == ObjectStatus.CONTACT_OPENING)
		{
			return true;
		}
		if (finger_b_object_status_ == ObjectStatus.CONTACT_CLOSING || finger_b_object_status_ == ObjectStatus.CONTACT_OPENING)
		{
			return true;
		}
		if (finger_c_object_status_ == ObjectStatus.CONTACT_CLOSING || finger_c_object_status_ == ObjectStatus.CONTACT_OPENING)
		{
			return true;
		}
		if (scissor_object_status_ == ObjectStatus.CONTACT_CLOSING || scissor_object_status_ == ObjectStatus.CONTACT_OPENING)
		{
			return true;
		}
		return false;
	}
	
	public boolean HasFault()
	{
		if (gripper_fault_status_ == GripperFaultStatus.NO_FAULTS)
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	public InitializationStatus getInitialization_status()
	{
		return initialization_status_;
	}

	public GripperActionStatus getGripper_action_status()
	{
		return gripper_action_status_;
	}

	public GripperSystemStatus getGripper_system_status()
	{
		return gripper_system_status_;
	}

	public GripperMotionStatus getGripper_motion_status()
	{
		return gripper_motion_status_;
	}

	public GripperFaultStatus getGripper_fault_status()
	{
		return gripper_fault_status_;
	}

	public ObjectStatus getFinger_a_object_status()
	{
		return finger_a_object_status_;
	}

	public ObjectStatus getFinger_b_object_status()
	{
		return finger_b_object_status_;
	}

	public ObjectStatus getFinger_c_object_status()
	{
		return finger_c_object_status_;
	}

	public ObjectStatus getScissor_object_status()
	{
		return scissor_object_status_;
	}

	public double getFinger_a_position_request()
	{
		return finger_a_position_request_;
	}

	public double getFinger_a_position()
	{
		return finger_a_position_;
	}

	public double getFinger_a_current()
	{
		return finger_a_current_;
	}

	public double getFinger_b_position_request()
	{
		return finger_b_position_request_;
	}

	public double getFinger_b_position()
	{
		return finger_b_position_;
	}

	public double getFinger_b_current()
	{
		return finger_b_current_;
	}

	public double getFinger_c_position_request()
	{
		return finger_c_position_request_;
	}

	public double getFinger_c_position()
	{
		return finger_c_position_;
	}

	public double getFinger_c_current()
	{
		return finger_c_current_;
	}

	public double getScissor_position_request()
	{
		return scissor_position_request_;
	}

	public double getScissor_position()
	{
		return scissor_position_;
	}

	public double getScissor_current()
	{
		return scissor_current_;
	}

}
