package armlab.demos;


import javax.inject.Inject;

import armlab.robotiq.gripper.Robotiq3FingerGripper;
import armlab.robotiq.gripper.Robotiq3FingerGripperCommand;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

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
public class ProspectiveStudentDemoLeftArm extends RoboticsAPIApplication
{
	@Inject
	private LBR iiwa7_arm_;
	private Robotiq3FingerGripper gripper_;
	
	private final Robotiq3FingerGripperCommand fully_open_gripper_config = new Robotiq3FingerGripperCommand(0.0, 0.0, 0.0, 1.0, 0.1, 0.0);
	private final Robotiq3FingerGripperCommand open_gripper_config = new Robotiq3FingerGripperCommand(0.25, 0.25, 0.25, 1.0, 0.1, 0.0);
	private final Robotiq3FingerGripperCommand closed_gripper_config = new Robotiq3FingerGripperCommand(0.6, 0.6, 0.6, 1.0, 0.1, 0.0);

	@Override
	public void initialize()
	{
		getLogger().info("Initializing...");
		gripper_ = new Robotiq3FingerGripper(iiwa7_arm_, getLogger());
		gripper_.InitializeGripper();
	}
	
	@Override
	public void run()
	{
		// your application execution starts here
		gripper_.CommandGripperBlocking(open_gripper_config);
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/Pickup_1")));
		gripper_.CommandGripperBlocking(closed_gripper_config);
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/ClearMonitors_2")));
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/AvoidJointLimit_3")));
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/AbovePlate_4")));
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/Release_5")));
		gripper_.CommandGripperBlocking(open_gripper_config);
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/AboveRelease_6")));
		gripper_.CommandGripperBlocking(fully_open_gripper_config);
//		iiwa7_arm.move(ptp(getApplicationData().getFrame("/ElbowDown_7")));
//		iiwa7_arm.move(ptp(getApplicationData().getFrame("/HandUp_8")));
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/FingersUp_9")));
		iiwa7_arm_.move(ptp(getApplicationData().getFrame("/EndPose_10")));		
	}
}