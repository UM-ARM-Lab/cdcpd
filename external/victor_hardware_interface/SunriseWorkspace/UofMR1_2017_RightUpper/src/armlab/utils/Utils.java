package armlab.utils;

import java.util.concurrent.TimeUnit;

import armlab.lcm.msgs.cartesian_control_mode_limits;
import armlab.lcm.msgs.cartesian_path_execution_parameters;
import armlab.lcm.msgs.cartesian_value_quantity;
import armlab.lcm.msgs.joint_path_execution_parameters;

public class Utils
{
	public static void wait(final int milliseconds)
	{
		try
		{
		    TimeUnit.MILLISECONDS.sleep(milliseconds);
		}
		catch (InterruptedException e)
		{}
	}
	
	public static double getUTCTimeAsDouble()
	{
		return (double) System.currentTimeMillis() / 1000.0;
	}
	
	public static boolean areEqual(joint_path_execution_parameters p1, joint_path_execution_parameters p2)
	{
		return (p1.joint_relative_acceleration == p2.joint_relative_acceleration) &&
				(p1.joint_relative_velocity == p2.joint_relative_velocity) &&
				(p1.override_joint_acceleration == p2.override_joint_acceleration);
	}
	
	public static boolean areEqual(cartesian_value_quantity q1, cartesian_value_quantity q2)
	{
		return (q1.a == q2.a) && (q1.b == q2.b) && (q1.c == q2.c) && 
				(q1.x == q2.x) && (q1.y == q2.y) && (q1.z == q2.z);
	}
	
	public static boolean areEqual(cartesian_path_execution_parameters p1, cartesian_path_execution_parameters p2)
	{
		return areEqual(p1.max_acceleration, p2.max_acceleration) &&
				areEqual(p1.max_velocity, p2.max_velocity) &&
				p1.max_nullspace_velocity == p2.max_nullspace_velocity && 
				p1.max_nullspace_acceleration == p2.max_nullspace_acceleration;
	}
	
	public static boolean areEqual(cartesian_control_mode_limits l1, cartesian_control_mode_limits l2)
	{
		return areEqual(l1.max_cartesian_velocity, l2.max_cartesian_velocity) &&
				areEqual(l1.max_control_force, l2.max_control_force) &&
				areEqual(l1.max_path_deviation, l2.max_path_deviation) &&
				(l1.stop_on_max_control_force == l2.stop_on_max_control_force);
	}
}
