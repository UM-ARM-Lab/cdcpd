package com.kuka.generated.ioAccess;

import javax.inject.Inject;
import javax.inject.Singleton;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>Robotiq3Finger</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * 3 finger gripper
 */
@Singleton
public class Robotiq3FingerIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'Robotiq3Finger'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'Robotiq3Finger'
	 */
	@Inject
	public Robotiq3FingerIOGroup(Controller controller)
	{
		super(controller, "Robotiq3Finger");

		addDigitalOutput("ActionRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("GripperOptions", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("GripperOptions2", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("ThumbPositionRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("ThumbSpeedRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("ThumbForceRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("FirstFingerPositionRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("FirstFingerSpeedRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("FirstFingerForceRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("MiddleFingerPositionRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("MiddleFingerSpeedRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("MiddleFingerForceRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("ScissorPositionRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("ScissorSpeedRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addDigitalOutput("ScissorForceRequest", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("GripperStatus", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("ObjectStatus", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("FaultStatus", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("ThumbPositionRequestEcho", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("ThumbPosition", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("ThumbCurrent", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("FirstFingerPositionRequestEcho", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("FirstFingerPosition", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("FirstFingerCurrent", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("MidFingerPositionRequestEcho", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("MidFingerPosition", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("MidFingerCurrent", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("ScissorPositionRequestEcho", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("ScissorPosition", IOTypes.UNSIGNED_INTEGER, 8);
		addInput("ScissorCurrent", IOTypes.UNSIGNED_INTEGER, 8);
	}

	/**
	 * Gets the value of the <b>digital output '<i>ActionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'ActionRequest'
	 */
	public java.lang.Integer getActionRequest()
	{
		return getNumberIOValue("ActionRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>ActionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'ActionRequest'
	 */
	public void setActionRequest(java.lang.Integer value)
	{
		setDigitalOutput("ActionRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>GripperOptions</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'GripperOptions'
	 */
	public java.lang.Integer getGripperOptions()
	{
		return getNumberIOValue("GripperOptions", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>GripperOptions</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'GripperOptions'
	 */
	public void setGripperOptions(java.lang.Integer value)
	{
		setDigitalOutput("GripperOptions", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>GripperOptions2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'GripperOptions2'
	 */
	public java.lang.Integer getGripperOptions2()
	{
		return getNumberIOValue("GripperOptions2", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>GripperOptions2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'GripperOptions2'
	 */
	public void setGripperOptions2(java.lang.Integer value)
	{
		setDigitalOutput("GripperOptions2", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>ThumbPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'ThumbPositionRequest'
	 */
	public java.lang.Integer getThumbPositionRequest()
	{
		return getNumberIOValue("ThumbPositionRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>ThumbPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'ThumbPositionRequest'
	 */
	public void setThumbPositionRequest(java.lang.Integer value)
	{
		setDigitalOutput("ThumbPositionRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>ThumbSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'ThumbSpeedRequest'
	 */
	public java.lang.Integer getThumbSpeedRequest()
	{
		return getNumberIOValue("ThumbSpeedRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>ThumbSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'ThumbSpeedRequest'
	 */
	public void setThumbSpeedRequest(java.lang.Integer value)
	{
		setDigitalOutput("ThumbSpeedRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>ThumbForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'ThumbForceRequest'
	 */
	public java.lang.Integer getThumbForceRequest()
	{
		return getNumberIOValue("ThumbForceRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>ThumbForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'ThumbForceRequest'
	 */
	public void setThumbForceRequest(java.lang.Integer value)
	{
		setDigitalOutput("ThumbForceRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>FirstFingerPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'FirstFingerPositionRequest'
	 */
	public java.lang.Integer getFirstFingerPositionRequest()
	{
		return getNumberIOValue("FirstFingerPositionRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>FirstFingerPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'FirstFingerPositionRequest'
	 */
	public void setFirstFingerPositionRequest(java.lang.Integer value)
	{
		setDigitalOutput("FirstFingerPositionRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>FirstFingerSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'FirstFingerSpeedRequest'
	 */
	public java.lang.Integer getFirstFingerSpeedRequest()
	{
		return getNumberIOValue("FirstFingerSpeedRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>FirstFingerSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'FirstFingerSpeedRequest'
	 */
	public void setFirstFingerSpeedRequest(java.lang.Integer value)
	{
		setDigitalOutput("FirstFingerSpeedRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>FirstFingerForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'FirstFingerForceRequest'
	 */
	public java.lang.Integer getFirstFingerForceRequest()
	{
		return getNumberIOValue("FirstFingerForceRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>FirstFingerForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'FirstFingerForceRequest'
	 */
	public void setFirstFingerForceRequest(java.lang.Integer value)
	{
		setDigitalOutput("FirstFingerForceRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>MiddleFingerPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'MiddleFingerPositionRequest'
	 */
	public java.lang.Integer getMiddleFingerPositionRequest()
	{
		return getNumberIOValue("MiddleFingerPositionRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>MiddleFingerPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'MiddleFingerPositionRequest'
	 */
	public void setMiddleFingerPositionRequest(java.lang.Integer value)
	{
		setDigitalOutput("MiddleFingerPositionRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>MiddleFingerSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'MiddleFingerSpeedRequest'
	 */
	public java.lang.Integer getMiddleFingerSpeedRequest()
	{
		return getNumberIOValue("MiddleFingerSpeedRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>MiddleFingerSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'MiddleFingerSpeedRequest'
	 */
	public void setMiddleFingerSpeedRequest(java.lang.Integer value)
	{
		setDigitalOutput("MiddleFingerSpeedRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>MiddleFingerForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'MiddleFingerForceRequest'
	 */
	public java.lang.Integer getMiddleFingerForceRequest()
	{
		return getNumberIOValue("MiddleFingerForceRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>MiddleFingerForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'MiddleFingerForceRequest'
	 */
	public void setMiddleFingerForceRequest(java.lang.Integer value)
	{
		setDigitalOutput("MiddleFingerForceRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>ScissorPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'ScissorPositionRequest'
	 */
	public java.lang.Integer getScissorPositionRequest()
	{
		return getNumberIOValue("ScissorPositionRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>ScissorPositionRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'ScissorPositionRequest'
	 */
	public void setScissorPositionRequest(java.lang.Integer value)
	{
		setDigitalOutput("ScissorPositionRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>ScissorSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'ScissorSpeedRequest'
	 */
	public java.lang.Integer getScissorSpeedRequest()
	{
		return getNumberIOValue("ScissorSpeedRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>ScissorSpeedRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'ScissorSpeedRequest'
	 */
	public void setScissorSpeedRequest(java.lang.Integer value)
	{
		setDigitalOutput("ScissorSpeedRequest", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>ScissorForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital output 'ScissorForceRequest'
	 */
	public java.lang.Integer getScissorForceRequest()
	{
		return getNumberIOValue("ScissorForceRequest", true).intValue();
	}

	/**
	 * Sets the value of the <b>digital output '<i>ScissorForceRequest</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'ScissorForceRequest'
	 */
	public void setScissorForceRequest(java.lang.Integer value)
	{
		setDigitalOutput("ScissorForceRequest", value);
	}

	/**
	 * Gets the value of the <b>digital input '<i>GripperStatus</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'GripperStatus'
	 */
	public java.lang.Integer getGripperStatus()
	{
		return getNumberIOValue("GripperStatus", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ObjectStatus</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'ObjectStatus'
	 */
	public java.lang.Integer getObjectStatus()
	{
		return getNumberIOValue("ObjectStatus", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>FaultStatus</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'FaultStatus'
	 */
	public java.lang.Integer getFaultStatus()
	{
		return getNumberIOValue("FaultStatus", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ThumbPositionRequestEcho</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'ThumbPositionRequestEcho'
	 */
	public java.lang.Integer getThumbPositionRequestEcho()
	{
		return getNumberIOValue("ThumbPositionRequestEcho", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ThumbPosition</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'ThumbPosition'
	 */
	public java.lang.Integer getThumbPosition()
	{
		return getNumberIOValue("ThumbPosition", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ThumbCurrent</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'ThumbCurrent'
	 */
	public java.lang.Integer getThumbCurrent()
	{
		return getNumberIOValue("ThumbCurrent", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>FirstFingerPositionRequestEcho</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'FirstFingerPositionRequestEcho'
	 */
	public java.lang.Integer getFirstFingerPositionRequestEcho()
	{
		return getNumberIOValue("FirstFingerPositionRequestEcho", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>FirstFingerPosition</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'FirstFingerPosition'
	 */
	public java.lang.Integer getFirstFingerPosition()
	{
		return getNumberIOValue("FirstFingerPosition", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>FirstFingerCurrent</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'FirstFingerCurrent'
	 */
	public java.lang.Integer getFirstFingerCurrent()
	{
		return getNumberIOValue("FirstFingerCurrent", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>MidFingerPositionRequestEcho</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'MidFingerPositionRequestEcho'
	 */
	public java.lang.Integer getMidFingerPositionRequestEcho()
	{
		return getNumberIOValue("MidFingerPositionRequestEcho", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>MidFingerPosition</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'MidFingerPosition'
	 */
	public java.lang.Integer getMidFingerPosition()
	{
		return getNumberIOValue("MidFingerPosition", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>MidFingerCurrent</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'MidFingerCurrent'
	 */
	public java.lang.Integer getMidFingerCurrent()
	{
		return getNumberIOValue("MidFingerCurrent", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ScissorPositionRequestEcho</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'ScissorPositionRequestEcho'
	 */
	public java.lang.Integer getScissorPositionRequestEcho()
	{
		return getNumberIOValue("ScissorPositionRequestEcho", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ScissorPosition</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'ScissorPosition'
	 */
	public java.lang.Integer getScissorPosition()
	{
		return getNumberIOValue("ScissorPosition", false).intValue();
	}

	/**
	 * Gets the value of the <b>digital input '<i>ScissorCurrent</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0; 255]
	 *
	 * @return current value of the digital input 'ScissorCurrent'
	 */
	public java.lang.Integer getScissorCurrent()
	{
		return getNumberIOValue("ScissorCurrent", false).intValue();
	}

}
