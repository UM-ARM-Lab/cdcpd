package armlab.robotiq.gripper;

import armlab.lcm.msgs.*;

public class Robotiq3FingerGripperCommand
{
	private double finger_a_position_;
	private double finger_a_speed_;
	private double finger_a_force_;
	private double finger_b_position_;
	private double finger_b_speed_;
	private double finger_b_force_;
	private double finger_c_position_;
	private double finger_c_speed_;
	private double finger_c_force_;
	private double scissor_position_;
	private double scissor_speed_;
	private double scissor_force_;
	
	private static double LimitCommandValue(double raw_command)
	{
		if (raw_command < 0.0)
		{
			return 0.0;
		}
		else if (raw_command > 1.0)
		{
			return 1.0;
		}
		else
		{
			return raw_command;
		}
	}
	
	public static int ConvertToCommandValue(double command)
	{
		double value = 255.0 * command;
		int command_value = (int)value;
		if (command_value < 0)
		{
			return 0;
		}
		else if (command_value > 255)
		{
			return 255;
		}
		else
		{
			return command_value;
		}
	}
	
	public Robotiq3FingerGripperCommand(final robotiq_3finger_command cmd)
	{
		setFinger_a_position(cmd.finger_a_command.position);
		setFinger_a_speed(cmd.finger_a_command.speed);
		setFinger_a_force(cmd.finger_a_command.force);
		setFinger_b_position(cmd.finger_b_command.position);
		setFinger_b_speed(cmd.finger_b_command.speed);
		setFinger_b_force(cmd.finger_b_command.force);
		setFinger_c_position(cmd.finger_c_command.position);
		setFinger_c_speed(cmd.finger_c_command.speed);
		setFinger_c_force(cmd.finger_c_command.force);
		setScissor_position(cmd.scissor_command.position);
		setScissor_speed(cmd.scissor_command.speed);
		setScissor_force(cmd.scissor_command.force);
	}
	
	public Robotiq3FingerGripperCommand(double finger_a_position, double finger_a_speed, double finger_a_force,
										double finger_b_position, double finger_b_speed, double finger_b_force,
										double finger_c_position, double finger_c_speed, double finger_c_force,
										double scissor_position, double scissor_speed, double scissor_force)
	{
		setFinger_a_position(finger_a_position);
		setFinger_a_speed(finger_a_speed);
		setFinger_a_force(finger_a_force);
		setFinger_b_position(finger_b_position);
		setFinger_b_speed(finger_b_speed);
		setFinger_b_force(finger_b_force);
		setFinger_c_position(finger_c_position);
		setFinger_c_speed(finger_c_speed);
		setFinger_c_force(finger_c_force);
		setScissor_position(scissor_position);
		setScissor_speed(scissor_speed);
		setScissor_force(scissor_force);
	}
	
	public Robotiq3FingerGripperCommand(double finger_a_position,
										double finger_b_position,
										double finger_c_position,
										double finger_speed,
										double finger_force,
										double scissor_position,
										double scissor_speed,
										double scissor_force)
	{
		setFinger_a_position(finger_a_position);
		setFinger_a_speed(finger_speed);
		setFinger_a_force(finger_speed);
		setFinger_b_position(finger_b_position);
		setFinger_b_speed(finger_speed);
		setFinger_b_force(finger_speed);
		setFinger_c_position(finger_c_position);
		setFinger_c_speed(finger_speed);
		setFinger_c_force(finger_speed);
		setScissor_position(scissor_position);
		setScissor_speed(scissor_speed);
		setScissor_force(scissor_force);
	}
	
	public Robotiq3FingerGripperCommand(double finger_a_position,
										double finger_b_position,
										double finger_c_position,
										double scissor_position,
										double speed,
										double force)
	{
		setFinger_a_position(finger_a_position);
		setFinger_a_speed(speed);
		setFinger_a_force(force);
		setFinger_b_position(finger_b_position);
		setFinger_b_speed(speed);
		setFinger_b_force(force);
		setFinger_c_position(finger_c_position);
		setFinger_c_speed(speed);
		setFinger_c_force(force);
		setScissor_position(scissor_position);
		setScissor_speed(speed);
		setScissor_force(force);
	}
	
	public Robotiq3FingerGripperCommand(double finger_a_position,
										double finger_b_position,
										double finger_c_position,
										double scissor_position)
	{
		setFinger_a_position(finger_a_position);
		setFinger_a_speed(0.0);
		setFinger_a_force(0.0);
		setFinger_b_position(finger_b_position);
		setFinger_b_speed(0.0);
		setFinger_b_force(0.0);
		setFinger_c_position(finger_c_position);
		setFinger_c_speed(0.0);
		setFinger_c_force(0.0);
		setScissor_position(scissor_position);
		setScissor_speed(0.0);
		setScissor_force(0.0);
	}
	
	public Robotiq3FingerGripperCommand(double finger_a_position,
										double finger_b_position,
										double finger_c_position)
	{
		setFinger_a_position(finger_a_position);
		setFinger_a_speed(0.0);
		setFinger_a_force(0.0);
		setFinger_b_position(finger_b_position);
		setFinger_b_speed(0.0);
		setFinger_b_force(0.0);
		setFinger_c_position(finger_c_position);
		setFinger_c_speed(0.0);
		setFinger_c_force(0.0);
		setScissor_position(140.0 / 255.0);
		setScissor_speed(0.0);
		setScissor_force(0.0);
	}
	
	public double getFinger_a_position()
	{
		return finger_a_position_;
	}
	
	public int getFinger_a_position_command()
	{
		return ConvertToCommandValue(finger_a_position_);
	}
	
	public void setFinger_a_position(double finger_a_position)
	{
		this.finger_a_position_ = LimitCommandValue(finger_a_position);
	}
	
	public double getFinger_a_speed()
	{
		return finger_a_speed_;
	}
	
	public int getFinger_a_speed_command()
	{
		return ConvertToCommandValue(finger_a_speed_);
	}
	
	public void setFinger_a_speed(double finger_a_speed)
	{
		this.finger_a_speed_ = LimitCommandValue(finger_a_speed);
	}
	
	public double getFinger_a_force()
	{
		return finger_a_force_;
	}
	
	public int getFinger_a_force_command()
	{
		return ConvertToCommandValue(finger_a_force_);
	}
	
	public void setFinger_a_force(double finger_a_force)
	{
		this.finger_a_force_ = LimitCommandValue(finger_a_force);
	}
	
	public double getFinger_b_position()
	{
		return finger_b_position_;
	}
	
	public int getFinger_b_position_command()
	{
		return ConvertToCommandValue(finger_b_position_);
	}
	
	public void setFinger_b_position(double finger_b_position)
	{
		this.finger_b_position_ = LimitCommandValue(finger_b_position);
	}
	
	public double getFinger_b_speed()
	{
		return finger_b_speed_;
	}
	
	public int getFinger_b_speed_command()
	{
		return ConvertToCommandValue(finger_b_speed_);
	}
	
	public void setFinger_b_speed(double finger_b_speed)
	{
		this.finger_b_speed_ = LimitCommandValue(finger_b_speed);
	}
	
	public double getFinger_b_force()
	{
		return finger_b_force_;
	}
	
	public int getFinger_b_force_command()
	{
		return ConvertToCommandValue(finger_b_force_);
	}
	
	public void setFinger_b_force(double finger_b_force)
	{
		this.finger_b_force_ = LimitCommandValue(finger_b_force);
	}
	
	public double getFinger_c_position_request()
	{
		return finger_c_position_;
	}
	
	public int getFinger_c_position_command()
	{
		return ConvertToCommandValue(finger_c_position_);
	}
	
	public void setFinger_c_position(double finger_c_position)
	{
		this.finger_c_position_ = LimitCommandValue(finger_c_position);
	}
	
	public double getFinger_c_speed()
	{
		return finger_c_speed_;
	}
	
	public int getFinger_c_speed_command()
	{
		return ConvertToCommandValue(finger_c_speed_);
	}
	
	public void setFinger_c_speed(double finger_c_speed)
	{
		this.finger_c_speed_ = LimitCommandValue(finger_c_speed);
	}
	
	public double getFinger_c_force()
	{
		return finger_c_force_;
	}
	
	public int getFinger_c_force_command()
	{
		return ConvertToCommandValue(finger_c_force_);
	}
	
	public void setFinger_c_force(double finger_c_force)
	{
		this.finger_c_force_ = LimitCommandValue(finger_c_force);
	}
	
	public double getScissor_position()
	{
		return scissor_position_;
	}
	
	public int getScissor_position_command()
	{
		return ConvertToCommandValue(scissor_position_);
	}
	
	public void setScissor_position(double scissor_position)
	{
		this.scissor_position_ = LimitCommandValue(scissor_position);
	}
	
	public double getScissor_speed()
	{
		return scissor_speed_;
	}
	
	public int getScissor_speed_command()
	{
		return ConvertToCommandValue(scissor_speed_);
	}
	
	public void setScissor_speed(double scissor_speed)
	{
		this.scissor_speed_ = LimitCommandValue(scissor_speed);
	}
	
	public double getScissor_force()
	{
		return scissor_force_;
	}
	
	public int getScissor_force_command()
	{
		return ConvertToCommandValue(scissor_force_);
	}
	
	public void setScissor_force(double scissor_force)
	{
		this.scissor_force_ = LimitCommandValue(scissor_force);
	}
}
