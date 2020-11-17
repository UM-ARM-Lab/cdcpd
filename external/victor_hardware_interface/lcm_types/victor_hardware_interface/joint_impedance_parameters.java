/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package victor_hardware_interface;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class joint_impedance_parameters implements lcm.lcm.LCMEncodable
{
    public victor_hardware_interface.joint_value_quantity joint_stiffness;
    public victor_hardware_interface.joint_value_quantity joint_damping;
 
    public joint_impedance_parameters()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x183f83ab1b5b43a7L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(victor_hardware_interface.joint_impedance_parameters.class))
            return 0L;
 
        classes.add(victor_hardware_interface.joint_impedance_parameters.class);
        long hash = LCM_FINGERPRINT_BASE
             + victor_hardware_interface.joint_value_quantity._hashRecursive(classes)
             + victor_hardware_interface.joint_value_quantity._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        this.joint_stiffness._encodeRecursive(outs); 
 
        this.joint_damping._encodeRecursive(outs); 
 
    }
 
    public joint_impedance_parameters(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public joint_impedance_parameters(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static victor_hardware_interface.joint_impedance_parameters _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        victor_hardware_interface.joint_impedance_parameters o = new victor_hardware_interface.joint_impedance_parameters();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.joint_stiffness = victor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
        this.joint_damping = victor_hardware_interface.joint_value_quantity._decodeRecursiveFactory(ins);
 
    }
 
    public victor_hardware_interface.joint_impedance_parameters copy()
    {
        victor_hardware_interface.joint_impedance_parameters outobj = new victor_hardware_interface.joint_impedance_parameters();
        outobj.joint_stiffness = this.joint_stiffness.copy();
 
        outobj.joint_damping = this.joint_damping.copy();
 
        return outobj;
    }
 
}

