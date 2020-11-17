package armlab.robotiq.gripper;

import java.util.concurrent.TimeUnit;

import armlab.lcm.msgs.robotiq_3finger_actuator_status;
import armlab.lcm.msgs.robotiq_3finger_object_status;
import armlab.lcm.msgs.robotiq_3finger_status;
import armlab.utils.Utils;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.generated.ioAccess.Robotiq3FingerIOGroup;
import com.kuka.task.ITaskLogger;

public class Robotiq3FingerGripper
{
	private ITaskLogger logger_;
	private Robotiq3FingerIOGroup robotiq_3finger_IO_;
	
	private final static int rACT_RESET = 0x0 << 0;
	private final static int rACT_ACTIVATION = 0x1 << 0;
	private final static int rMOD_BASIC = 0x0 << 1;
	//private final static int rMOD_PINCH = 0x1 << 1;
	//private final static int rMOD_WIDE = 0x2 << 1;
	//private final static int rMOD_SCISSOR = 0x3 << 1;
	private final static int rGTO_STOPPED = 0x0 << 3;
	private final static int rGTO_GOTO = 0x1 << 3;
	private final static int rATR_NORMAL = 0x0 << 4;
	//private final static int rATR_EMERGENCY_AUTO_RELEASE = 0x1 << 4;
	private final static int rGLV_NO_GLOVE = 0x0 << 0;
	//private final static int rGLV_HAS_GLOVE = 0x1 << 0;
	//private final static int rICF_NORMAL = 0x0 << 2;
	private final static int rICF_INDEPENDENT = 0x1 << 2;
	//private final static int rICS_NORMAL = 0x0 << 3;
	private final static int rICS_INDEPENDENT = 0x1 << 3;
	
	public Robotiq3FingerGripper(LBR iiwa_arm, ITaskLogger logger)
	{
		logger_ = logger;
		robotiq_3finger_IO_ = new Robotiq3FingerIOGroup(iiwa_arm.getController());
	}
	
	public boolean ReinitializeGripper()
	{
		logger_.info("Reinitializing/activating the gripper...");
		// Stop the gripper
		robotiq_3finger_IO_.setActionRequest(rACT_RESET);
		// Set the mode flags to zero
		robotiq_3finger_IO_.setGripperOptions(0);
		// Start the activation
		robotiq_3finger_IO_.setActionRequest(rACT_ACTIVATION);
		// Wait for the activation to finish
		logger_.info("Waiting for gripper to activate...");
		do
		{
			try
			{
			    TimeUnit.SECONDS.sleep(1);
			}
			catch (InterruptedException e)
			{
				;
			}
		}
		while (GetGripperStatus().IsActivated() != true);
		// Make sure we activated with no faults
		final Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
		if (gripper_status.IsActivated() && (gripper_status.HasFault() == false))
		{
			logger_.info("...gripper activation finished successfully");
			return true;
		}
		else
		{
			logger_.info("...gripper activation failed or finished with a fault");
			return false;
		}
	}
	
	public boolean InitializeGripper()
	{
		final Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
		if (gripper_status.IsActivated())
		{
			logger_.info("Gripper is already initialized, no need to reactivate");
			return true;
		}
		else
		{
			return ReinitializeGripper();
		}
	}
	
	public boolean CommandGripper(Robotiq3FingerGripperCommand gripper_command)
	{
		final Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
		if (gripper_status.IsActivated())
		{
//			logger_.info("Commanding motion...");
			// Stop the gripper
			robotiq_3finger_IO_.setActionRequest(rACT_ACTIVATION | rMOD_BASIC | rGTO_STOPPED | rATR_NORMAL);
			// Set the mode flags
			robotiq_3finger_IO_.setGripperOptions(rGLV_NO_GLOVE | rICF_INDEPENDENT | rICS_INDEPENDENT);
			// Set the finger + scissor commands
			robotiq_3finger_IO_.setFirstFingerPositionRequest(gripper_command.getFinger_a_position_command());
			robotiq_3finger_IO_.setFirstFingerSpeedRequest(gripper_command.getFinger_a_speed_command());
			robotiq_3finger_IO_.setFirstFingerForceRequest(gripper_command.getFinger_a_force_command());
			robotiq_3finger_IO_.setMiddleFingerPositionRequest(gripper_command.getFinger_b_position_command());
			robotiq_3finger_IO_.setMiddleFingerSpeedRequest(gripper_command.getFinger_b_speed_command());
			robotiq_3finger_IO_.setMiddleFingerForceRequest(gripper_command.getFinger_b_force_command());
			robotiq_3finger_IO_.setThumbPositionRequest(gripper_command.getFinger_c_position_command());
			robotiq_3finger_IO_.setThumbSpeedRequest(gripper_command.getFinger_c_speed_command());
			robotiq_3finger_IO_.setThumbForceRequest(gripper_command.getFinger_c_force_command());
			robotiq_3finger_IO_.setScissorPositionRequest(gripper_command.getScissor_position_command());
			robotiq_3finger_IO_.setScissorSpeedRequest(gripper_command.getScissor_speed_command());
			robotiq_3finger_IO_.setScissorForceRequest(gripper_command.getScissor_force_command());
			// Restart the gripper
			robotiq_3finger_IO_.setActionRequest(rACT_ACTIVATION | rMOD_BASIC | rGTO_GOTO | rATR_NORMAL);
//			logger_.info("Motion command sent...");
			return true;
		}
		else
		{
			logger_.error("Gripper is not activated, cannot command motion");
			return false;
		}
	}
	
	public boolean CommandGripperBlocking(Robotiq3FingerGripperCommand gripper_command)
	{
		if (CommandGripper(gripper_command))
		{
//			logger_.info("Waiting for motion command to finish...");
			// Wait for the motion to finish
			do
			{
				try
				{
				    TimeUnit.SECONDS.sleep(1);
				}
				catch (InterruptedException e)
				{
					;
				}
			}
			while (GetGripperStatus().IsStopped() != true);
			// Make sure we finished with no faults
			final Robotiq3FingerGripperStatus gripper_status = GetGripperStatus();
			if (gripper_status.IsActivated() && (gripper_status.HasFault() == false))
			{
//				logger_.info("...motion command finished successfully");
				return true;
			}
			else
			{
//				logger_.error("...motion command failed or faulted");
				return false;
			}
		}
		else
		{
			return false;
		}
	}
	
	public Robotiq3FingerGripperStatus GetGripperStatus()
	{
		Robotiq3FingerGripperStatus gripper_status =
				new Robotiq3FingerGripperStatus(
				robotiq_3finger_IO_.getGripperStatus(),
				robotiq_3finger_IO_.getObjectStatus(),
				robotiq_3finger_IO_.getFaultStatus(),
				robotiq_3finger_IO_.getFirstFingerPositionRequestEcho(),
				robotiq_3finger_IO_.getFirstFingerPosition(),
				robotiq_3finger_IO_.getFirstFingerCurrent(),
				robotiq_3finger_IO_.getMidFingerPositionRequestEcho(),
				robotiq_3finger_IO_.getMidFingerPosition(),
				robotiq_3finger_IO_.getMidFingerCurrent(),
				robotiq_3finger_IO_.getThumbPositionRequestEcho(),
				robotiq_3finger_IO_.getThumbPosition(),
				robotiq_3finger_IO_.getThumbCurrent(),
				robotiq_3finger_IO_.getScissorPositionRequestEcho(),
				robotiq_3finger_IO_.getScissorPosition(),
				robotiq_3finger_IO_.getScissorCurrent());
		return gripper_status;
	}
	
	public void PopulateLCMStatusMessage(robotiq_3finger_status gripper_status)
	{
		final double now = Utils.getUTCTimeAsDouble();
		Robotiq3FingerGripperStatus.PopulateLCMStatusMessage(
				gripper_status, 
				now, 
				robotiq_3finger_IO_.getGripperStatus(),
				robotiq_3finger_IO_.getObjectStatus(),
				robotiq_3finger_IO_.getFaultStatus(),
				robotiq_3finger_IO_.getFirstFingerPositionRequestEcho(),
				robotiq_3finger_IO_.getFirstFingerPosition(),
				robotiq_3finger_IO_.getFirstFingerCurrent(),
				robotiq_3finger_IO_.getMidFingerPositionRequestEcho(),
				robotiq_3finger_IO_.getMidFingerPosition(),
				robotiq_3finger_IO_.getMidFingerCurrent(),
				robotiq_3finger_IO_.getThumbPositionRequestEcho(),
				robotiq_3finger_IO_.getThumbPosition(),
				robotiq_3finger_IO_.getThumbCurrent(),
				robotiq_3finger_IO_.getScissorPositionRequestEcho(),
				robotiq_3finger_IO_.getScissorPosition(),
				robotiq_3finger_IO_.getScissorCurrent());
	}
}
