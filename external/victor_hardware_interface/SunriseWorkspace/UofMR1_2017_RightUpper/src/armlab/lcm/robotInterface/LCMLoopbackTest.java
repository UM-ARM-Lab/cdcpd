package armlab.lcm.robotInterface;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;


import armlab.lcm.msgs.*;
import armlab.robotiq.gripper.Robotiq3FingerGripper;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import lcm.lcm.*;

public class LCMLoopbackTest extends RoboticsAPIApplication implements LCMSubscriber
{
	@Inject
	private LBR iiwa7_arm_;
	private Robotiq3FingerGripper gripper_;
	private LCM lcm_subscriber_;
	private LCM lcm_publisher_;
	
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
	{
		try 
		{
			if (channel.equals("motion_command"))
			{
				motion_command msg = new motion_command(ins);
//				getLogger().info(msg.toString());
				
				motion_status reply = new motion_status();
				reply.measured_joint_position = msg.joint_position;
				reply.commanded_joint_position = msg.joint_position;
				reply.measured_joint_velocity = msg.joint_velocity;
				reply.measured_joint_torque = new joint_value_quantity();
				reply.estimated_external_torque = new joint_value_quantity();

				reply.measured_cartesian_pose = msg.cartesian_pose;
				reply.commanded_cartesian_pose = msg.cartesian_pose;
				reply.estimated_external_wrench = new cartesian_value_quantity();
				
				reply.active_control_mode = msg.control_mode;
				
				reply.timestamp = msg.timestamp;
				
				lcm_publisher_.publish("motion_status", reply);
			}
			else if (channel.equals("control_mode_command"))
			{
				control_mode_parameters msg = new control_mode_parameters(ins);
//				getLogger().info(msg.toString());
				
				control_mode_parameters reply = new control_mode_parameters();
				reply.joint_impedance_params = msg.joint_impedance_params;
				reply.cartesian_impedance_params = msg.cartesian_impedance_params;
				reply.joint_path_execution_params = msg.joint_path_execution_params;
				reply.cartesian_path_execution_params = msg.cartesian_path_execution_params;
				reply.control_mode = msg.control_mode;
				
				reply.timestamp = msg.timestamp;
				
				lcm_publisher_.publish("control_mode_status", reply);
			}
			else if (channel.equals("gripper_command"))
			{
				robotiq_3finger_command msg = new robotiq_3finger_command(ins);
//				getLogger().info(msg.toString());
				
				robotiq_3finger_status reply = new robotiq_3finger_status();
				reply.finger_a_status = new robotiq_3finger_actuator_status();
				reply.finger_a_status.position_request = msg.finger_a_command.position;
				reply.finger_b_status = new robotiq_3finger_actuator_status();
				reply.finger_b_status.position_request = msg.finger_b_command.position;
				reply.finger_c_status = new robotiq_3finger_actuator_status();
				reply.finger_c_status.position_request = msg.finger_c_command.position;
				reply.scissor_status = new robotiq_3finger_actuator_status();
				reply.scissor_status.position_request = msg.scissor_command.position;
				
				reply.finger_a_object_status = new robotiq_3finger_object_status();
				reply.finger_b_object_status = new robotiq_3finger_object_status();
				reply.finger_c_object_status = new robotiq_3finger_object_status();
				reply.scissor_object_status = new robotiq_3finger_object_status();
				
				reply.timestamp = msg.timestamp;
				
				lcm_publisher_.publish("gripper_status", reply);
			}
			else
			{
				getLogger().warn("Unknown LCM channel: " + channel);
			}
		}
		catch (IOException ex)
		{
			getLogger().info("Exception: " + ex);
		}
	}
	
	@Override
	public void initialize()
	{		
		getLogger().info("Initializing Gripper");
		gripper_ = new Robotiq3FingerGripper(iiwa7_arm_, getLogger());
		gripper_.InitializeGripper();
		
		try
		{
			getLogger().info("Initializing LCM Publisher");
			lcm_publisher_ = new LCM(LCMURLs.DEFAULT_DEST_URL);
			
			getLogger().info("Initializing LCM Subscriptions");
			lcm_subscriber_ = new LCM(LCMURLs.DEFAULT_SELF_URL);
			lcm_subscriber_.subscribeAll(this);
		}
		catch (IOException ex)
		{
			getLogger().info("LCM Subscription Exception: " + ex);
		}
	}
	
	@Override
	public void run()
	{
		try
		{
		    while(true)
		    {
		    	TimeUnit.MILLISECONDS.sleep(1000);
		    }
	    }
		catch (InterruptedException ex)
		{}
		
	}
}
