package armlab.lcm.robotInterface;

import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import armlab.lcm.msgs.cartesian_control_mode_limits;
import armlab.lcm.msgs.cartesian_impedance_parameters;
import armlab.lcm.msgs.cartesian_path_execution_parameters;
import armlab.lcm.msgs.control_mode;
import armlab.lcm.msgs.control_mode_parameters;
import armlab.lcm.msgs.joint_impedance_parameters;
import armlab.lcm.msgs.joint_path_execution_parameters;
import armlab.lcm.msgs.motion_command;
import armlab.lcm.msgs.motion_status;
import armlab.lcm.msgs.robotiq_3finger_actuator_status;
import armlab.lcm.msgs.robotiq_3finger_command;
import armlab.lcm.msgs.robotiq_3finger_object_status;
import armlab.lcm.msgs.robotiq_3finger_status;
import armlab.robotiq.gripper.Robotiq3FingerGripper;
import armlab.robotiq.gripper.Robotiq3FingerGripperCommand;
import armlab.robotiq.gripper.Robotiq3FingerGripperStatus;
import armlab.utils.Utils;

import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.persistenceModel.PersistenceException;

public class LCMRobotInterface extends RoboticsAPIApplication implements LCMSubscriber
{
    static final public String CONTROL_MODE_COMMAND_CHANNEL = "control_mode_command";
    static final public String CONTROL_MODE_STATUS_CHANNEL = "control_mode_status";
    static final public String MOTION_COMMAND_CHANNEL = "motion_command";
    static final public String MOTION_STATUS_CHANNEL = "motion_status";
    static final public String GRIPPER_COMMAND_CHANNEL = "gripper_command";
    static final public String GRIPPER_STATUS_CHANNEL = "gripper_status";
    
    static final public int CONTROL_MODE_FEEDBACK_PERIOD_MS = 1000;
    static final public int MOTION_STATUS_FEEDBACK_PERIOD_MS = 10; 
    static final public int GRIPPER_FEEDBACK_PERIOD_MS = 100;
    static final public int MAIN_LOOP_CONTROL_PERIOD_MS = 10; 
    
    static final public double MINIMUM_TRAJECTORY_EXECUTION_TIME = 20e-3;
    static final public double TIMEOUT_AFTER_GOAL_REACH = 3600;
    
    @Inject
    private LBR iiwa7_arm_;
    private Robotiq3FingerGripper gripper_;
    private LCM lcm_subscriber_;
    private LCM lcm_publisher_;
    
    private Timer main_loop_timer_;
    private TimerTask main_loop_task_;
    
    private Object arm_io_lock_;
    private ControlModePublisher control_mode_publisher_;
    private MotionStatusPublisher motion_status_publisher_;
    private Robotiq3FingerGripperPublisher gripper_publisher_;
    
    private ControlModeCommandHandler control_mode_command_handler_;
    private MotionCommandHandler motion_command_handler_;
    private GripperCommandHandler gripper_command_handler_;
    
    
    private Tool tool_; //End Effector attached to the end of the robot"
    private ObjectFrame end_effector_frame_; 
    
    private boolean running_;

    // TODO remove these as member variables. These are properties of the ArmController
    private joint_path_execution_parameters joint_path_execution_params_;
    private cartesian_path_execution_parameters cartesian_path_execution_params_;
    private ArmController arm_controller_;

    
    @Override
    public void initialize()
    {
        
        try
        {
            tool_ = (Tool)getApplicationData().createFromTemplate("Robotiq3FingerGripper");
            tool_.attachTo(iiwa7_arm_.getFlange());
            end_effector_frame_ = tool_.getFrame("PalmSurface");
        }
        catch (PersistenceException ex)
        {
            getLogger().error("Robotiq3FingerGripper tool not available. Using flange.");
            end_effector_frame_ = iiwa7_arm_.getFlange();
        }
        
        if (end_effector_frame_ == null)
        {
            throw new IllegalArgumentException("Something is wrong with tools or frame names");
        }
        
        //Set valid data on initialization
        joint_path_execution_params_ = new joint_path_execution_parameters();
        joint_path_execution_params_.joint_relative_acceleration = 0.1;
        joint_path_execution_params_.joint_relative_velocity = 0.1;
        joint_path_execution_params_.override_joint_acceleration = 0.0;
        
        //Populate with valid data, even though this data is never used for control
        cartesian_path_execution_params_ = new cartesian_path_execution_parameters();
        cartesian_path_execution_params_.max_nullspace_velocity = 0.1;
        cartesian_path_execution_params_.max_velocity = Conversions.cvqInitializer(0.1);
        cartesian_path_execution_params_.max_nullspace_acceleration = 0.1;
        cartesian_path_execution_params_.max_acceleration = Conversions.cvqInitializer(0.1);
        
        control_mode_parameters cmd = new control_mode_parameters();
        cmd.joint_path_execution_params = joint_path_execution_params_;
        arm_controller_ = new JointPositionController(cmd);

        //This is needed to activate the smartservo motion
        //the Servo runtime object cannot be accessed before this
        end_effector_frame_.moveAsync(arm_controller_.getIMotion());
        
        
        arm_io_lock_ = new Object();
        
        if (tool_ != null)
        {
            getLogger().info("Initializing Gripper");
            gripper_ = new Robotiq3FingerGripper(iiwa7_arm_, getLogger());
            gripper_.InitializeGripper();
        }
        else
        {
            getLogger().info("No Gripper to initialize");
        }
        
        try
        {
            getLogger().info("Initializing LCM Publisher");
            lcm_publisher_ = new LCM(LCMURLs.DEFAULT_DEST_URL);
            control_mode_publisher_ = new ControlModePublisher();
            gripper_publisher_ = new Robotiq3FingerGripperPublisher();
            motion_status_publisher_ = new MotionStatusPublisher();
            
            getLogger().info("Initializing LCM Subscriptions");
            control_mode_command_handler_ = new ControlModeCommandHandler();
            motion_command_handler_ = new MotionCommandHandler();
            gripper_command_handler_ = new GripperCommandHandler();
            lcm_subscriber_ = new LCM(LCMURLs.DEFAULT_SELF_URL);
            lcm_subscriber_.subscribeAll(this);
        }
        catch (IOException ex)
        {
            getLogger().info("LCM Subscription Exception: " + ex);
        }
        
        

        getLogger().info("Creating control loop objects");
        main_loop_timer_ = new Timer ();
        main_loop_task_ = new TimerTask()
        {
            @Override
            public void run ()
            {
                control_mode_parameters control_mode_cmd;
                Targets targets = new Targets();
                synchronized (arm_io_lock_)
                {
                    control_mode_cmd = control_mode_command_handler_.getControlModeCommand();
                    targets.joint_position_target = motion_command_handler_.getJointPositionTarget();
                    targets.cartesian_pose_target = motion_command_handler_.getCartesianPoseTarget();
                    targets.joint_impedance_target = motion_command_handler_.getJointImpedanceTarget();
                    targets.cartesian_impedance_target = motion_command_handler_.getCartesianImpedanceTarget();
                }
                
                if (control_mode_cmd != null)
                {
                    switchControlMode(control_mode_cmd);
                }
                
                arm_controller_.setDestination(targets);
            }
        };
    }
    
    @Override
    public void run()
    {
        running_ = true;

        getLogger().info("Starting SmartServo motion");
        
        getLogger().info("Starting feedback threads");
        control_mode_publisher_.start();
        motion_status_publisher_.start();
        gripper_publisher_.start();


        getLogger().info("Starting control loop");
        main_loop_timer_.schedule(main_loop_task_, 0, MAIN_LOOP_CONTROL_PERIOD_MS);

        // Spin; doing nothing until we get interrupted for some reason
        try
        {
            while (running_)
            {
                TimeUnit.SECONDS.sleep(10);
            }
        }
        catch (InterruptedException ex)
        {
            getLogger().info("Main loop interrupted: " + ex);
        }
    }
    
    @Override
    public void dispose()
    {
        getLogger().info("Disposing...");
        running_ = false;
        main_loop_timer_.cancel();
        gripper_publisher_.cancel();
        motion_status_publisher_.cancel();
        control_mode_publisher_.cancel();
        
        lcm_publisher_.close();
        lcm_subscriber_.close();
        super.dispose();
    }
    
    @Override 
    public void onApplicationStateChanged(RoboticsAPIApplicationState state)
    {
        if (state == RoboticsAPIApplicationState.STOPPING)
        {
            running_ = false;
        }
        super.onApplicationStateChanged(state);
    };
    
    
    private SmartServo createSmartServoMotion(final joint_path_execution_parameters params)
    {
        SmartServo motion = new SmartServo(iiwa7_arm_.getCurrentJointPosition());
        motion.setMinimumTrajectoryExecutionTime(MINIMUM_TRAJECTORY_EXECUTION_TIME);
        motion.setJointVelocityRel(params.joint_relative_velocity);                    
        motion.setJointAccelerationRel(params.joint_relative_acceleration);
        motion.overrideJointAcceleration(params.override_joint_acceleration);
        motion.setTimeoutAfterGoalReach(TIMEOUT_AFTER_GOAL_REACH);
        
        return motion;
    }
    
    private SmartServoLIN createSmartServoLINMotion(final cartesian_path_execution_parameters params)
    {
        SmartServoLIN motion = new SmartServoLIN(iiwa7_arm_.getCurrentCartesianPosition(end_effector_frame_));
        motion.setMinimumTrajectoryExecutionTime(MINIMUM_TRAJECTORY_EXECUTION_TIME);
        motion.setMaxNullSpaceAcceleration(params.max_nullspace_acceleration);
        motion.setMaxNullSpaceVelocity(params.max_nullspace_velocity);
        motion.setMaxOrientationAcceleration(new double[]
            {params.max_acceleration.a, params.max_acceleration.b, params.max_acceleration.c});
        motion.setMaxOrientationVelocity(new double[]
            {params.max_velocity.a, params.max_velocity.b, params.max_velocity.c});

        //*1000 is conversion from m (LCM) to mm (Kuka)
        motion.setMaxTranslationAcceleration(new double[]
            {params.max_acceleration.x * 1000, params.max_acceleration.y * 1000, params.max_acceleration.z * 1000});
        motion.setMaxTranslationVelocity(new double[]
            {params.max_velocity.x, params.max_velocity.y, params.max_velocity.z});
        motion.setTimeoutAfterGoalReach(TIMEOUT_AFTER_GOAL_REACH);
        return motion;
    }
    
    @Override
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try 
        {
            if (channel.equals(CONTROL_MODE_COMMAND_CHANNEL))
            {
                control_mode_parameters cmd = new control_mode_parameters(ins);
                control_mode_command_handler_.storeControlModeCommand(cmd);
            }
            else if (channel.equals(MOTION_COMMAND_CHANNEL))
            {
                motion_command cmd = new motion_command(ins);
                motion_command_handler_.storeMotionCommand(cmd);
            }            
            else if (channel.equals(GRIPPER_COMMAND_CHANNEL))
            {
                robotiq_3finger_command cmd = new robotiq_3finger_command(ins);
                gripper_command_handler_.executeCommand(cmd);
            }
            else
            {
                getLogger().warn("Unknown LCM channel: " + channel);
            }
        }
        catch (IOException ex)
        {
            getLogger().info("Message received exception: " + ex);
        }
    }
    
    /**
     * Stops current motion and rebuild a new controller
     * 
     * @param cmd control mode along with necessary parameters
     * 
     */
    public void switchControlMode(control_mode_parameters cmd)
    {
        //getLogger().info("Changing control mode");
        synchronized (arm_io_lock_)
        { 
           //Don't rebuild control mode if we don't have to
            if(arm_controller_.canUpdate(cmd))
            {
                getLogger().info("Updating Control Mode: " + cmd.control_mode.mode);
                arm_controller_.update(cmd);
            }
            else 
            {
                buildControlMode(cmd);
            }
        }
    }
    
    public void buildControlMode(control_mode_parameters cmd)
    {
        getLogger().info("Switch to ControlMode: " + cmd.control_mode.mode);
        boolean stopped = arm_controller_.stop();
        assert(stopped);
        
        switch (cmd.control_mode.mode)
        {
            case(control_mode.JOINT_POSITION):
            {
                arm_controller_ = new JointPositionController(cmd);
                break;
            }
            case(control_mode.JOINT_IMPEDANCE):
            {
                arm_controller_ = new JointImpedanceController(cmd);
                break;
            }
            case(control_mode.CARTESIAN_POSE):
            {
                arm_controller_ = new CartesianPoseController(cmd);
                break;
            }
            case(control_mode.CARTESIAN_IMPEDANCE):
            {
                arm_controller_ = new CartesianImpedanceController(cmd);
                break;
            }
            default:
            {
                getLogger().error("Unknown control mode from LCM: " + cmd.control_mode);
            }
       
        }
        // Update params
        joint_path_execution_params_ = cmd.joint_path_execution_params;
        cartesian_path_execution_params_ = cmd.cartesian_path_execution_params;

        //This is needed to activate the smartservo motion
        //the Servo runtime object cannot be accessed before this
        end_effector_frame_.moveAsync(arm_controller_.getIMotion());
    }
    
    
    public class Targets
    {
        public JointPosition joint_position_target;
        public Frame cartesian_pose_target;
        public JointPosition joint_impedance_target;
        public Frame cartesian_impedance_target;
    }
    
    private abstract class ArmController
    {
        control_mode active_control_mode_;
        abstract void setDestination(Targets t);
        void update(control_mode_parameters cmd) {}
        boolean canUpdate(control_mode_parameters cmd)
        {
            return cmd.control_mode.mode == this.active_control_mode_.mode;
        }
        
        abstract boolean stop();
        abstract IMotion getIMotion();
        abstract void populateStatusMsg(control_mode_parameters control_mode_status_msg);
    }
    
    private abstract class JointController extends ArmController
    {
        public SmartServo joint_smartservo_motion_;
        
        @Override 
        IMotion getIMotion(){
            return joint_smartservo_motion_;
        }
        
        @Override
        boolean stop()
        {
            return joint_smartservo_motion_.getRuntime().stopMotion();
        }

        @Override
        boolean canUpdate(control_mode_parameters cmd)
        {
            return super.canUpdate(cmd) &&
                Utils.areEqual(cmd.joint_path_execution_params, joint_path_execution_params_);
        }
    }
    
    private class JointPositionController extends JointController
    {
        public JointPositionController(control_mode_parameters cmd) {
            
            //getLogger().info("Building new Joint Position control mode");
            joint_smartservo_motion_ = createSmartServoMotion(cmd.joint_path_execution_params);
            joint_smartservo_motion_.setMode(new PositionControlMode(true));
            active_control_mode_ = new control_mode();
            active_control_mode_.mode = control_mode.JOINT_POSITION;
            getLogger().info("Built new Joint Position control mode");
        }

        @Override
        void setDestination(Targets targets)
        {
            if (targets.joint_position_target != null)
            {
                // Without isReadyToMove() this code seems to block on setDestination
                if (iiwa7_arm_.isReadyToMove())
                {
                    joint_smartservo_motion_.getRuntime().setDestination(targets.joint_position_target);
                }
            }
        }
        
        @Override
        void populateStatusMsg(control_mode_parameters control_mode_status_msg)
        {
            // Path execution params
            control_mode_status_msg.joint_path_execution_params.joint_relative_acceleration = joint_smartservo_motion_.getJointAccelerationRel();
            control_mode_status_msg.joint_path_execution_params.joint_relative_velocity = joint_smartservo_motion_.getJointVelocityRel();
            control_mode_status_msg.joint_path_execution_params.override_joint_acceleration = joint_path_execution_params_.override_joint_acceleration;    
        }
    }
    
    private class JointImpedanceController extends JointController
    {
        public JointImpedanceControlMode jcm_;
        
        public JointImpedanceController(control_mode_parameters cmd) {
            //getLogger().info("Building new Joint Impedance control mode");
            active_control_mode_ = new control_mode();
            active_control_mode_.mode = control_mode.JOINT_IMPEDANCE;
            if (tool_ != null)
            {
                getLogger().info("Attempting to validate for impedance mode (arm + tool)");
                boolean validated = SmartServo.validateForImpedanceMode(tool_);
                assert(validated);
            }
            else
            {
                getLogger().info("Attempting to validate for impedance mode (arm only)");
                boolean validated = SmartServo.validateForImpedanceMode(iiwa7_arm_);
                assert(validated);
            }
            
            joint_smartservo_motion_ = createSmartServoMotion(cmd.joint_path_execution_params);
            
            jcm_ = new JointImpedanceControlMode(iiwa7_arm_.getJointCount());
            setStiffness(cmd);
            joint_smartservo_motion_.setMode(jcm_);
            getLogger().info("Built new Joint Impedance control mode");
        }
        
        @Override
        void update(control_mode_parameters cmd){
            setStiffness(cmd);
            joint_smartservo_motion_.getRuntime().changeControlModeSettings(jcm_);
        }
        
        void setStiffness(control_mode_parameters cmd) {
            jcm_.setDamping(Conversions.jvqToVector(cmd.joint_impedance_params.joint_damping));
            jcm_.setStiffness(Conversions.jvqToVector(cmd.joint_impedance_params.joint_stiffness));    
        }
    

        @Override
        void setDestination(Targets targets)
        {
            if (targets.joint_impedance_target != null)
            {
                // Without isReadyToMove() this code seems to block on setDestination
                if (iiwa7_arm_.isReadyToMove())
                {
                    joint_smartservo_motion_.getRuntime().setDestination(targets.joint_impedance_target);
                }
            }
        }
        
        @Override
        void populateStatusMsg(control_mode_parameters control_mode_status_msg)
        {
            // Impedance params
            JointImpedanceControlMode jcm = (JointImpedanceControlMode)joint_smartservo_motion_.getMode();
            Conversions.vectorToJvq(jcm.getDamping(), control_mode_status_msg.joint_impedance_params.joint_damping);
            Conversions.vectorToJvq(jcm.getStiffness(), control_mode_status_msg.joint_impedance_params.joint_stiffness);
            // Path execution params
            control_mode_status_msg.joint_path_execution_params.joint_relative_acceleration = joint_smartservo_motion_.getJointAccelerationRel();
            control_mode_status_msg.joint_path_execution_params.joint_relative_velocity = joint_smartservo_motion_.getJointVelocityRel();
            control_mode_status_msg.joint_path_execution_params.override_joint_acceleration = joint_path_execution_params_.override_joint_acceleration;
        }
    }
    
    private abstract class CartesianController extends ArmController
    {
        public SmartServoLIN cartesian_smartservo_motion_;
        
        @Override 
        IMotion getIMotion(){
            return cartesian_smartservo_motion_;
        }
        
        @Override
        boolean stop()
        {
            return cartesian_smartservo_motion_.getRuntime().stopMotion();
        }

        @Override
        boolean canUpdate(control_mode_parameters cmd)
        {
            return super.canUpdate(cmd) &&
                Utils.areEqual(cmd.cartesian_path_execution_params, cartesian_path_execution_params_) &&
                Utils.areEqual(cmd.cartesian_control_mode_limits, Conversions.ccmToControlModeLimits((CartesianImpedanceControlMode)cartesian_smartservo_motion_.getMode()));
        }

    }
    
    private class CartesianPoseController extends CartesianController
    {
        public CartesianPoseController(control_mode_parameters cmd) {
            //getLogger().info("Building new Cartesian Pose control mode");
            active_control_mode_ = new control_mode();
            active_control_mode_.mode = control_mode.CARTESIAN_POSE;
            cartesian_smartservo_motion_ = createSmartServoLINMotion(cmd.cartesian_path_execution_params);
            cartesian_smartservo_motion_.setMode(new PositionControlMode(true));
            getLogger().info("Built new Cartesian Pose control mode");
        }

        @Override
        void setDestination(Targets targets)
        {
            if (targets.cartesian_pose_target != null)
            {
                // Without isReadyToMove() this code seems to block on setDestination
                if (iiwa7_arm_.isReadyToMove())
                {
                    try
                    {
                        cartesian_smartservo_motion_.getRuntime().setDestination(targets.cartesian_pose_target);
                    }
                    catch(Exception e)
                    {
                        getLogger().error(e.getClass().getName() + ": " + e.getMessage());
                    } 
                }
            }
        }
        
        @Override
        void populateStatusMsg(control_mode_parameters control_mode_status_msg)
        {
            control_mode_status_msg.cartesian_path_execution_params = cartesian_path_execution_params_;
        }
    }
    
    private class CartesianImpedanceController extends CartesianController
    {

        public CartesianImpedanceController(control_mode_parameters cmd) {
            //getLogger().info("Building new Cartesian Impedance control mode");
            active_control_mode_ = new control_mode();
            active_control_mode_.mode = control_mode.CARTESIAN_IMPEDANCE;
            
            if (tool_ != null)
            {
                getLogger().info("Attempting to validate for impedance mode (arm + tool)");
                boolean validated = SmartServoLIN.validateForImpedanceMode(tool_);
                assert(validated);
            }
            else
            {
                getLogger().info("Attempting to validate for impedance mode (arm only)");
                boolean validated = SmartServoLIN.validateForImpedanceMode(iiwa7_arm_);
                assert(validated);
            }

            cartesian_smartservo_motion_ = createSmartServoLINMotion(cmd.cartesian_path_execution_params);
            
            CartesianImpedanceControlMode ccm = new CartesianImpedanceControlMode();
            ccm.parametrize(CartDOF.X).setDamping(cmd.cartesian_impedance_params.cartesian_damping.x);
            ccm.parametrize(CartDOF.Y).setDamping(cmd.cartesian_impedance_params.cartesian_damping.y);
            ccm.parametrize(CartDOF.Z).setDamping(cmd.cartesian_impedance_params.cartesian_damping.z);
            ccm.parametrize(CartDOF.A).setDamping(cmd.cartesian_impedance_params.cartesian_damping.a);
            ccm.parametrize(CartDOF.B).setDamping(cmd.cartesian_impedance_params.cartesian_damping.b);
            ccm.parametrize(CartDOF.C).setDamping(cmd.cartesian_impedance_params.cartesian_damping.c);
            ccm.parametrize(CartDOF.X).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.x);
            ccm.parametrize(CartDOF.Y).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.y);
            ccm.parametrize(CartDOF.Z).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.z);
            ccm.parametrize(CartDOF.A).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.a);
            ccm.parametrize(CartDOF.B).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.b);
            ccm.parametrize(CartDOF.C).setStiffness(cmd.cartesian_impedance_params.cartesian_stiffness.c);
            ccm.setNullSpaceDamping(cmd.cartesian_impedance_params.nullspace_damping);
            ccm.setNullSpaceStiffness(cmd.cartesian_impedance_params.nullspace_stiffness);
            //*1000 is conversion from m (LCM) to mm (Kuka)
            ccm.setMaxPathDeviation(cmd.cartesian_control_mode_limits.max_path_deviation.x * 1000,
                                    cmd.cartesian_control_mode_limits.max_path_deviation.y * 1000,
                                    cmd.cartesian_control_mode_limits.max_path_deviation.z * 1000,
                                    cmd.cartesian_control_mode_limits.max_path_deviation.a,
                                    cmd.cartesian_control_mode_limits.max_path_deviation.b,
                                    cmd.cartesian_control_mode_limits.max_path_deviation.c);
            ccm.setMaxCartesianVelocity(cmd.cartesian_control_mode_limits.max_cartesian_velocity.x * 1000,
                                        cmd.cartesian_control_mode_limits.max_cartesian_velocity.y * 1000,
                                        cmd.cartesian_control_mode_limits.max_cartesian_velocity.z * 1000,
                                        cmd.cartesian_control_mode_limits.max_cartesian_velocity.a,
                                        cmd.cartesian_control_mode_limits.max_cartesian_velocity.b,
                                        cmd.cartesian_control_mode_limits.max_cartesian_velocity.c);
            ccm.setMaxControlForce(cmd.cartesian_control_mode_limits.max_control_force.x,
                                   cmd.cartesian_control_mode_limits.max_control_force.y,
                                   cmd.cartesian_control_mode_limits.max_control_force.z,
                                   cmd.cartesian_control_mode_limits.max_control_force.a,
                                   cmd.cartesian_control_mode_limits.max_control_force.b,
                                   cmd.cartesian_control_mode_limits.max_control_force.c,
                                   cmd.cartesian_control_mode_limits.stop_on_max_control_force);
            
            cartesian_smartservo_motion_.setMode(ccm);
            getLogger().info("Built new Cartesian Impedance control mode");
        }

        @Override
        void setDestination(Targets targets)
        {
            if (targets.cartesian_impedance_target != null)
            {
                // Without isReadyToMove() this code seems to block on setDestination
                if (iiwa7_arm_.isReadyToMove())
                {
                    try
                    {
                        cartesian_smartservo_motion_.getRuntime().setDestination(targets.cartesian_impedance_target);
                    }
                    catch(Exception e)
                    {
                        getLogger().error(e.getClass().getName() + ": " + e.getMessage());
                    }
                    
                }
            }
        }
        
        @Override
        void populateStatusMsg(control_mode_parameters control_mode_status_msg)
        {
            CartesianImpedanceControlMode ccm = (CartesianImpedanceControlMode)cartesian_smartservo_motion_.getMode();
            // Impedance params
            Conversions.vectorToCvq(ccm.getDamping(), control_mode_status_msg.cartesian_impedance_params.cartesian_damping, false);
            Conversions.vectorToCvq(ccm.getStiffness(), control_mode_status_msg.cartesian_impedance_params.cartesian_stiffness, false);
            control_mode_status_msg.cartesian_impedance_params.nullspace_damping = ccm.getNullSpaceDamping();
            control_mode_status_msg.cartesian_impedance_params.nullspace_stiffness = ccm.getNullSpaceStiffness();
            // Cartesian control mode limits
            Conversions.vectorToCvq(ccm.getMaxCartesianVelocity(), control_mode_status_msg.cartesian_control_mode_limits.max_cartesian_velocity, true);
            Conversions.vectorToCvq(ccm.getMaxPathDeviation(), control_mode_status_msg.cartesian_control_mode_limits.max_path_deviation, true);
            Conversions.vectorToCvq(ccm.getMaxControlForce(), control_mode_status_msg.cartesian_control_mode_limits.max_control_force, false);
            control_mode_status_msg.cartesian_control_mode_limits.stop_on_max_control_force = ccm.hasMaxControlForceStopCondition();
        }
    }

    
    private class ControlModePublisher
    {
        private final control_mode_parameters control_mode_status_msg_;
        
        private final Timer timer_;
        private final TimerTask feedback_loop_task_;
        
        public ControlModePublisher()
        {
            final boolean is_daemon = true; 
            timer_ = new Timer(is_daemon);        
        
            // We need to make sure these get initialized into a valid state!
            // The numbers here are meaningless, we are going to get new values as soon as the program starts.
            control_mode_status_msg_ = new control_mode_parameters();
            control_mode_status_msg_.joint_impedance_params = new joint_impedance_parameters();
            control_mode_status_msg_.joint_impedance_params.joint_stiffness = Conversions.jvqInitializer(0.0);
            control_mode_status_msg_.joint_impedance_params.joint_damping = Conversions.jvqInitializer(0.0);
            
            control_mode_status_msg_.cartesian_impedance_params = new cartesian_impedance_parameters();
            control_mode_status_msg_.cartesian_impedance_params.cartesian_stiffness = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_impedance_params.nullspace_damping = 0.3;
            control_mode_status_msg_.cartesian_impedance_params.cartesian_damping = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_impedance_params.nullspace_stiffness = 0.0;
            
            control_mode_status_msg_.cartesian_control_mode_limits = new cartesian_control_mode_limits();
            control_mode_status_msg_.cartesian_control_mode_limits.max_cartesian_velocity = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_control_mode_limits.max_control_force = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_control_mode_limits.max_path_deviation = Conversions.cvqInitializer(0.1);
            control_mode_status_msg_.cartesian_control_mode_limits.stop_on_max_control_force = false;
            
            //These numbers must be set by the encapsulating class.
            control_mode_status_msg_.joint_path_execution_params = joint_path_execution_params_;
            control_mode_status_msg_.cartesian_path_execution_params = cartesian_path_execution_params_;
            
            control_mode_status_msg_.control_mode = new control_mode();
            
            feedback_loop_task_ = new TimerTask()
            {
                @Override
                public void run ()
                {
                    synchronized(arm_io_lock_)
                    {
                        final double now = Utils.getUTCTimeAsDouble();
                        control_mode_status_msg_.timestamp = now;
                        control_mode_status_msg_.control_mode.mode = arm_controller_.active_control_mode_.mode;
                        // Populate everything by default
                        control_mode_status_msg_.joint_path_execution_params = joint_path_execution_params_;
                        control_mode_status_msg_.cartesian_path_execution_params = cartesian_path_execution_params_;
                        // Get running path execution & impedance params if possible
                        
                        arm_controller_.populateStatusMsg(control_mode_status_msg_);
                        
                    }                    
                    lcm_publisher_.publish(CONTROL_MODE_STATUS_CHANNEL, control_mode_status_msg_);
                }
            };
        }
        
        public void start()
        {
            // schedule the task to run now, and then every T milliseconds
            timer_.schedule(feedback_loop_task_, 0, CONTROL_MODE_FEEDBACK_PERIOD_MS);
        }
        
        public void cancel()
        {
            timer_.cancel();
        }
    }
        
    private class MotionStatusPublisher
    {
        private final motion_status motion_status_msg_;
        
        private final Timer timer_;
        private final TimerTask feedback_loop_task_;
        
        public MotionStatusPublisher()
        {
            motion_status_msg_ = new motion_status();
            motion_status_msg_.measured_joint_position = Conversions.jvqInitializer(0.0);
            motion_status_msg_.commanded_joint_position = Conversions.jvqInitializer(0.0);
            motion_status_msg_.measured_joint_velocity = Conversions.jvqInitializer(0.0);
            motion_status_msg_.measured_joint_torque = Conversions.jvqInitializer(0.0);
            motion_status_msg_.estimated_external_torque = Conversions.jvqInitializer(0.0);
            motion_status_msg_.measured_cartesian_pose_abc = Conversions.cvqInitializer(0.0);
            motion_status_msg_.commanded_cartesian_pose_abc = Conversions.cvqInitializer(0.0);
            motion_status_msg_.measured_cartesian_pose = Conversions.identityPose();
            motion_status_msg_.commanded_cartesian_pose = Conversions.identityPose();
            motion_status_msg_.estimated_external_wrench = Conversions.cvqInitializer(0.0);
            motion_status_msg_.active_control_mode = new control_mode();
            
            timer_ = new Timer();
            feedback_loop_task_ = new TimerTask()
            {
                @Override
                public void run ()
                {
                    synchronized (arm_io_lock_)
                    {
                        final double now = Utils.getUTCTimeAsDouble();
                        motion_status_msg_.timestamp = now;
                        motion_status_msg_.active_control_mode.mode = arm_controller_.active_control_mode_.mode;
                        Conversions.jointPositionToJvq(iiwa7_arm_.getCurrentJointPosition(), motion_status_msg_.measured_joint_position);
                        Conversions.jointPositionToJvq(iiwa7_arm_.getCommandedJointPosition(), motion_status_msg_.commanded_joint_position);
                        Conversions.jvqInitializer(0.0, motion_status_msg_.measured_joint_velocity); // No joint velocity data exists natively
                        Conversions.vectorToJvq(iiwa7_arm_.getMeasuredTorque().getTorqueValues(), motion_status_msg_.measured_joint_torque);
                        Conversions.vectorToJvq(iiwa7_arm_.getExternalTorque().getTorqueValues(), motion_status_msg_.estimated_external_torque);
                        Transformation commanded_world_ee = iiwa7_arm_.getCommandedCartesianPosition(end_effector_frame_).transformationFromWorld();
                        Transformation measured_world_ee = iiwa7_arm_.getCurrentCartesianPosition(end_effector_frame_).transformationFromWorld();
                        Conversions.transformationToCvq(measured_world_ee, motion_status_msg_.measured_cartesian_pose_abc);
                        Conversions.transformationToCvq(commanded_world_ee, motion_status_msg_.commanded_cartesian_pose_abc);
                        Conversions.transformationToPose(measured_world_ee, motion_status_msg_.measured_cartesian_pose);
                        Conversions.transformationToPose(commanded_world_ee, motion_status_msg_.commanded_cartesian_pose);
                        Conversions.forceTorqueToCvq(iiwa7_arm_.getExternalForceTorque(end_effector_frame_), motion_status_msg_.estimated_external_wrench);
                    }
                    lcm_publisher_.publish(MOTION_STATUS_CHANNEL, motion_status_msg_);
                }
            };
        }
        
        public void start()
        {
            // schedule the task to run now, and then every T milliseconds
            timer_.schedule(feedback_loop_task_, 0, MOTION_STATUS_FEEDBACK_PERIOD_MS);
        }
        
        public void cancel()
        {
            timer_.cancel();
        }
    }
        
    private class Robotiq3FingerGripperPublisher
    {
        private final robotiq_3finger_status gripper_status_msg_;
        
        private final Timer timer_;
        private final TimerTask feedback_loop_task_;
        
        public Robotiq3FingerGripperPublisher()
        {
            // Double check some assertions regarding message definitions and internal representations
            assert((byte) Robotiq3FingerGripperStatus.InitializationStatus.GRIPPER_RESET.ordinal() == robotiq_3finger_status.GRIPPER_RESET);
            assert((byte) Robotiq3FingerGripperStatus.InitializationStatus.GRIPPER_ACTIVATION.ordinal() == robotiq_3finger_status.GRIPPER_ACTIVATION);
            
            assert((byte) Robotiq3FingerGripperStatus.GripperActionStatus.GRIPPER_STOPPED_OR_BUSY.ordinal() == robotiq_3finger_status.GRIPPER_STOPPED_OR_BUSY);
            assert((byte) Robotiq3FingerGripperStatus.GripperActionStatus.GRIPPER_GOTO.ordinal() == robotiq_3finger_status.GRIPPER_GOTO);
    
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_RESET_OR_AUTO_RELEASE.ordinal() == robotiq_3finger_status.GRIPPER_RESET_OR_AUTO_RELEASE);
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_ACTIVATION_IN_PROGRESS.ordinal() == robotiq_3finger_status.GRIPPER_ACTIVATION_IN_PROGRESS);
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_MODE_CHANGE_IN_PROGESS.ordinal() == robotiq_3finger_status.GRIPPER_MODE_CHANGE_IN_PROGRESS);
            assert((byte) Robotiq3FingerGripperStatus.GripperSystemStatus.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE.ordinal() == robotiq_3finger_status.GRIPPER_ACTIVATION_MODE_CHANGE_COMPLETE);
            
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_STOPPED_UNKNOWN.ordinal() == robotiq_3finger_status.GRIPPER_STOPPED_UNKNOWN);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_IN_MOTION.ordinal() == robotiq_3finger_status.GRIPPER_IN_MOTION);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_ONE_OR_TWO_STOPPED_EARLY.ordinal() == robotiq_3finger_status.GRIPPER_ONE_OR_TWO_STOPPED_EARLY);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_ALL_STOPPED_EARLY.ordinal() == robotiq_3finger_status.GRIPPER_ALL_STOPPED_EARLY);
            assert((byte) Robotiq3FingerGripperStatus.GripperMotionStatus.GRIPPER_ALL_AT_REQUESTED.ordinal() == robotiq_3finger_status.GRIPPER_ALL_AT_REQUESTED);
    
            // TODO: These assertion need to be here, but the enum doesn't match. Need to fix this when we do the gripper status code review.
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.STOPPED.ordinal() == robotiq_3finger_object_status.STOPPED);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.IN_MOTION.ordinal() == robotiq_3finger_object_status.IN_MOTION);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.CONTACT_OPENING.ordinal() == robotiq_3finger_object_status.CONTACT_OPENING);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.CONTACT_CLOSING.ordinal() == robotiq_3finger_object_status.CONTACT_CLOSING);
//            assert((byte) Robotiq3FingerGripperStatus.ObjectStatus.AT_REQUESTED.ordinal() == robotiq_3finger_object_status.AT_REQUESTED);
            // TODO: The assertion for the GripperFaultStatus check also need to be here, but as the current code directly return the values from robotiq_3finger_status, we don't need this now.
            // TODO: The enum of GripperFaultStatus also doesn't match the robotiq_3finger_status.
            gripper_status_msg_ = new robotiq_3finger_status();
            
            gripper_status_msg_.finger_a_status = new robotiq_3finger_actuator_status();
            gripper_status_msg_.finger_b_status = new robotiq_3finger_actuator_status();
            gripper_status_msg_.finger_c_status = new robotiq_3finger_actuator_status();
            gripper_status_msg_.scissor_status = new robotiq_3finger_actuator_status();
            
            gripper_status_msg_.finger_a_object_status = new robotiq_3finger_object_status();
            gripper_status_msg_.finger_b_object_status = new robotiq_3finger_object_status();
            gripper_status_msg_.finger_c_object_status = new robotiq_3finger_object_status();
            gripper_status_msg_.scissor_object_status = new robotiq_3finger_object_status();
            
            final boolean is_daemon = true; 
            timer_ = new Timer(is_daemon);
            
            feedback_loop_task_ = new TimerTask ()
            {
                @Override
                public void run ()
                {
                    if (tool_ != null)
                    {
                        synchronized(gripper_)
                        {
                            gripper_.PopulateLCMStatusMessage(gripper_status_msg_);
                        }
                        lcm_publisher_.publish(GRIPPER_STATUS_CHANNEL, gripper_status_msg_);
                    }
                }
            };
        }
        
        public void start()
        {
            // schedule the task to run now, and then every T milliseconds
            timer_.schedule(feedback_loop_task_, 0, GRIPPER_FEEDBACK_PERIOD_MS);
        }
    
        public void cancel()
        {
            timer_.cancel();
        }
    }

    /**
     * This class acts as a buffer to store control mode command message    
     * @author armlab
     *
     */
    private class ControlModeCommandHandler
    {
        private control_mode_parameters control_mode_command_;
        private Boolean new_control_mode_command_ready_ = new Boolean(false);
        
        public void storeControlModeCommand(control_mode_parameters cmd)
        {
            synchronized (new_control_mode_command_ready_)
            {
                control_mode_command_ = cmd;
                new_control_mode_command_ready_ = true;
            }
        }
        
        public control_mode_parameters getControlModeCommand()
        {
            synchronized (new_control_mode_command_ready_)
            {
                if (new_control_mode_command_ready_)
                {
                    new_control_mode_command_ready_ = false;
                    return control_mode_command_;
                }
                else
                {
                    return null;
                }
            }
        }
    }

    /**
     * This class acts as a buffer to store motion command message
     * @author armlab
     *
     */
    private class MotionCommandHandler
    {
 
        private control_mode control_mode_ = null;
        private final JointPosition joint_position_target_ = new JointPosition(iiwa7_arm_.getJointCount());
        private Frame cartesian_pose_target_ = new Frame();
        private Boolean new_motion_command_ready_ = new Boolean(false);
        
        public void storeMotionCommand(motion_command cmd)
        {
            synchronized (new_motion_command_ready_)
            {
                control_mode_ = cmd.control_mode; 
                switch (cmd.control_mode.mode)
                {
                    case control_mode.JOINT_POSITION:
                    case control_mode.JOINT_IMPEDANCE:
                    {
                        storeJointPositionCommand(cmd);
                        break;
                    }
                    case control_mode.CARTESIAN_POSE:
                    case control_mode.CARTESIAN_IMPEDANCE:
                    {
                        storeCartesianPoseCommand(cmd);
                        break;
                    }
                    default:
                    {
                        getLogger().error("MotionMode: " + cmd.control_mode + " is invalid");
                        break;
                    }
                }
            }
        }
    
        private void storeJointPositionCommand(motion_command cmd)
        {
            synchronized (new_motion_command_ready_)
            {
                Conversions.jvqToJointPosition(cmd.joint_position, joint_position_target_);
                new_motion_command_ready_ = true;
            }
        }
    
        private void storeCartesianPoseCommand(motion_command cmd)
        {
            synchronized (new_motion_command_ready_)
            {    
                // The output of this function is inherently in WorldFrame,
                // so we don't need to convert it to a different parent frame
                // as this is the same frame used by cartesian_smartservo_motion_
                // in the encapsulating class
                cartesian_pose_target_ = Conversions.poseToFrame(cmd.cartesian_pose);
                new_motion_command_ready_ = true;
            }
        }
        
        public JointPosition getJointPositionTarget()
        {
            synchronized (new_motion_command_ready_)
            {
                if (new_motion_command_ready_ && control_mode_.mode == control_mode.JOINT_POSITION)
                {
                    new_motion_command_ready_ = false;
                    return joint_position_target_;
                }
                else
                {
                    return null;
                }
            }
        }
        
        public Frame getCartesianPoseTarget()
        {
            synchronized (new_motion_command_ready_)
            {
                if (new_motion_command_ready_ && control_mode_.mode == control_mode.CARTESIAN_POSE)
                {
                    new_motion_command_ready_ =  false;
                    return cartesian_pose_target_;
                }
                else
                {
                    return null;
                }
            }
        }
        
        public JointPosition getJointImpedanceTarget()
        {
            synchronized (new_motion_command_ready_)
            {
                if (new_motion_command_ready_ && control_mode_.mode == control_mode.JOINT_IMPEDANCE)
                {
                    new_motion_command_ready_ = false;
                    return joint_position_target_;
                }
                else
                {
                    return null;
                }
            }
        }
        
        public Frame getCartesianImpedanceTarget()
        {
            synchronized (new_motion_command_ready_)
            {
                if (new_motion_command_ready_ && control_mode_.mode == control_mode.CARTESIAN_IMPEDANCE)
                {
                    new_motion_command_ready_ = false;
                    return cartesian_pose_target_;
                }
                else
                {
                    return null;
                }
            }
        }
    }
    
    /**
     * This class will command the gripper after receiving a gripper command message. 
     * We may need further discussion on this class about what behavior we want for the gripper.
     * @author armlab
     *
     */
    private class GripperCommandHandler
    {
        private void executeCommand(robotiq_3finger_command cmd)
        {
            if (tool_ != null)
            {
                synchronized(gripper_)
                {
                    gripper_.CommandGripper(new Robotiq3FingerGripperCommand(cmd));
                }
            }
        }
    }
}


