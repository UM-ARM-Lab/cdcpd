package armlab.demos;


import javax.inject.Inject;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.motionModel.CartesianPTP;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;

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
public class GravityCompensation extends RoboticsAPIApplication
{
	@Inject
	private LBR iiwa7_arm;

	@Override
	public void initialize()
	{
		getLogger().info("Initializing...");
	}
	
	@Override
	public void run()
	{
		getLogger().info("Move to start position");
		CartesianPTP ptpToStartPosition = ptp(getApplicationData().getFrame("/GravityCompHome"));
		ptpToStartPosition.setJointVelocityRel(0.25);
		iiwa7_arm.move(ptpToStartPosition);
		
		
		
		final double stiffness = 100;
		final double damping = 0.9;
		final double max_linear_vel = 1500; // mm/s
		final double max_angular_vel = 1.0; // rad/s
		final double max_linear_force = 5;
		final double max_angular_force = 0.5;
		
		CartesianImpedanceControlMode impedanceControlMode = new CartesianImpedanceControlMode();
		
		impedanceControlMode.parametrize(CartDOF.X).setStiffness(stiffness);
		impedanceControlMode.parametrize(CartDOF.Y).setStiffness(stiffness);
		impedanceControlMode.parametrize(CartDOF.Z).setStiffness(300);
		impedanceControlMode.parametrize(CartDOF.TRANSL).setDamping(damping);
		impedanceControlMode.parametrize(CartDOF.ROT).setStiffness(stiffness/10);
		impedanceControlMode.parametrize(CartDOF.ROT).setDamping(damping);
		
		impedanceControlMode.setMaxCartesianVelocity(max_linear_vel, max_linear_vel, max_linear_vel, max_angular_vel, max_angular_vel, max_angular_vel);		
		impedanceControlMode.setMaxControlForce(max_linear_force, max_linear_force, 50, max_angular_force, max_angular_force, max_angular_force, false);

		
		IMotionContainer positionHoldContainer = iiwa7_arm.moveAsync((new PositionHold(impedanceControlMode, -1, null)));

		getLogger().info("Show modal dialog while executing position hold");
		getApplicationUI().displayModalDialog(ApplicationDialogType.INFORMATION, "Press ok to finish the application.", "OK");

		// As soon as the modal dialog returns, the motion container will be cancelled. This finishes the position hold. 
		positionHoldContainer.cancel();
	
		getLogger().info("Operation Mode:" + iiwa7_arm.getOperationMode());
		
		getLogger().info("External Torque:" + iiwa7_arm.getExternalForceTorque(iiwa7_arm.getFlange(), World.Current.getRootFrame()));
		getLogger().info("Raw Torque:" + iiwa7_arm.getMeasuredTorque());
		
//		getLogger().info("Load on Flange:" + iiwa7_arm.calculateLoadOnFlange());
		
	}
}