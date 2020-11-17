package armlab.demos;

import javax.inject.Inject;
import java.util.concurrent.TimeUnit;
import armlab.robotiq.gripper.*;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import com.kuka.roboticsAPI.deviceModel.LBR;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class ProspectiveStudentDemoRightArm extends RoboticsAPIApplication
{
	@Inject
	private LBR iiwa7_arm;
	private Robotiq3FingerGripper gripper_;
	
	private final Robotiq3FingerGripperCommand plate_initial_gripper_config_ = new Robotiq3FingerGripperCommand(20.0/255.0, 90.0/255.0, 90.0/255.0, 140.0/255.0, 0.1, 0.0);
	private final Robotiq3FingerGripperCommand plate_grasp_gripper_config_ = new Robotiq3FingerGripperCommand(255.0/255.0, 110.0/255.0, 110.0/255.0, 140.0/255.0, 0.1, 1.0);
	private final Robotiq3FingerGripperCommand plate_release_gripper_config_ = new Robotiq3FingerGripperCommand(110.0/255.0, 60.0/255.0, 60.0/255.0, 140.0/255.0, 0.02, 0.0);
	private final Robotiq3FingerGripperCommand bottle_open_gripper_config_ = new Robotiq3FingerGripperCommand(0.0/255.0, 0.0/255.0, 0.0/255.0, 140.0/255.0, 0.2, 0.8);
	private final Robotiq3FingerGripperCommand bottle_close_gripper_config_ = new Robotiq3FingerGripperCommand(255.0/255.0, 255.0/255.0, 255.0/255.0, 140.0/255.0, 0.2, 0.8);
	@Override
	public void initialize()
	{
		getLogger().info("Initializing...");
		gripper_ = new Robotiq3FingerGripper(iiwa7_arm, getLogger());
		gripper_.InitializeGripper();
	}
	
	public void wait(final int seconds)
	{
		try
		{
		    TimeUnit.SECONDS.sleep(seconds);
		}
		catch (InterruptedException e)
		{
			;
		}
	}
	

	@Override
	public void run()
	{
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Home")));
		gripper_.CommandGripper(plate_initial_gripper_config_);
		Plate();
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Home")));
		gripper_.CommandGripper(bottle_open_gripper_config_);
		wait(3);
		Bottle();
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Home")));
		gripper_.CommandGripper(plate_initial_gripper_config_);
	}
	
	public void Bottle()
	{
//		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Bottle1")));// This is the last pose from Plate
//		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Bottle2")));
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Bottle3")));
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Bottle4")));
		gripper_.CommandGripperBlocking(bottle_open_gripper_config_);
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Bottle5")));
		gripper_.CommandGripperBlocking(bottle_close_gripper_config_);
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Bottle6")));
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Bottle7_new")));
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Bottle8_new")));
		gripper_.CommandGripperBlocking(bottle_open_gripper_config_);
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Bottle9_new")));
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Home")));
	}
	
	public void Plate()
	{
		gripper_.CommandGripper(plate_initial_gripper_config_);
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Plate1")));
		gripper_.CommandGripperBlocking(plate_initial_gripper_config_);
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Plate2")));
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Plate3")).setJointVelocityRel(0.5));
		gripper_.CommandGripperBlocking(plate_grasp_gripper_config_);
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Plate4")));
//		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Plate5")));
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Plate6")));
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Plate7")).setJointVelocityRel(0.5));
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Plate8_new")));
		gripper_.CommandGripperBlocking(plate_release_gripper_config_);
		iiwa7_arm.move(ptp(getApplicationData().getFrame("/Plate9_new")));
		iiwa7_arm.move(lin(getApplicationData().getFrame("/Plate10_new")));
	}
}