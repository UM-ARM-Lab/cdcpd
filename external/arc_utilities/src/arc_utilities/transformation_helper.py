#!/usr/bin/env python

#   Calder Phillips-Grafflin -- ARC Lab
#
#   Various helper functions for operations with transformations
#   using quaternions + vectors and 4x4 matrices
#
####################################################################################################
#
# A note to the user - this code was written because the last time I needed to do one of the
# operations provided in this API (composing a pose with a transform), it required ~20 lines of code
#
# It relies exclusively on the powerful tranformations.py library and numpy linear algebra library
#
####################################################################################################

<<<<<<< HEAD
import numpy as np
from numpy import sin, cos

from geometry_msgs.msg import Point, Quaternion, Transform, Pose
=======
import rospy
import math
import numpy
from numpy import *
from tf.transformations import *
from geometry_msgs.msg import *

>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
####################################################################################################
#
#   API Documentation
#
#   Storage conventions:
#
#   translations = [x,y,z]
#   quaternions = [x,y,z,w]
#
##################################################
# Math functions
#
# geometry_msgs/Transform composed = ComposeTransforms(geometry_msgs/Transform transform1, geometry_msgs/Transform transform2)
# geometry_msgs/Pose composed = ComposePoses(geometry_msgs/Pose pose1, geometry_msgs/Pose pose2)
#
# geometry_msgs/Transform inverted = InvertTransform(geometry_msgs/Transform old_transform)
# geometry_msgs/Pose inverted = InvertPose(geometry_msgs/Pose old_pose)
#
# numpy/array composed = ComposeMatrices(numpy/array matrix1, numpy/array matrix2)
# numpy/array inverted = InvertMatrix(numpy/array old_matrix)
#
##################################################
# Assist functions
#
# list quaternion = NormalizeQuaternion(list quaternion)
#
# geometry_msgs/Quaternion quaternion = NormalizeQuaternionRos(geometry_msgs/Quaternion quaternion)
#
##################################################
# Conversion functions
#
# geometry_msgs/Transform new_transform = TransformFromComponents(list translation, list quaternion)
# [list translation, list quaternion] = ComponentsFromTransform(geometry_msgs/Transform old_transform)
#
# geometry_msgs/Pose new_pose = PoseFromTransform(geometry_msgs/Transform old_transform)
# geometry_msgs/Transform new_transform = PoseToTransform(geometry_msgs/Pose old_pose)
#
# numpy/array tfmatrix = TransformToMatrix(geometry_msgs/Transform old_transform)
# geometry_msgs/Transform new_transform = TransformFromMatrix(numpy/array old_matrix)
#
# numpy/array tfmatrix = PoseToMatrix(geometry_msgs/Pose old_pose)
# geometry_msgs/Pose new_pose = PoseFromMatrix(numpy/array old_matrix)
#
# [numpy/array rotation, list translation] = ExtractRawFromMatrix(numpy/array old_matrix)
# numpy/array tfmatrix = BuildRawMatrix(numpy/array rotation, list translation)
#
# [list translation, list quaternion] = ExtractFromMatrix(numpy/array old_matrix)
# numpy/array tfmatrix = BuildMatrix(list translation, list quaternion)
#
##################################################
# Generation functions -- these generate various transform representations from D-H parameters
#                         we recommend against using these functions unless necessary
#
# numpy/array tfmatrix = BuildMatrixFromDH(float d, float a, float theta, float alpha)
# [list translation, list quaternion] = ExtractFromDH(float d, float a, float theta, float alpha)
# geometry_msgs/Transform new_transform = TransformFromDH(float d, float a, float theta, float alpha)
# geometry_msgs/Pose new_pose = PoseFromDH(float d, float a, float theta, float alpha)
#
####################################################################################################
<<<<<<< HEAD
from tf.transformations import quaternion_from_matrix, quaternion_matrix
=======
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default

'''High-level functions'''


def AddPoints(point1, point2):
    sum_point = Point()
    sum_point.x = point2.x + point1.x
    sum_point.y = point2.y + point1.y
    sum_point.z = point2.z + point1.z
    return sum_point


def SubtractPoints(point1, point2):
    difference = Point()
    difference.x = point2.x - point1.x
    difference.y = point2.y - point1.y
    difference.z = point2.z - point1.z
    return difference


def TranslationNorm(trans):
<<<<<<< HEAD
    return np.sqrt(trans.x ** 2 + trans.y ** 2 + trans.z ** 2)
=======
    return math.sqrt(trans.x**2 + trans.y**2 + trans.z**2)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default


def ComposePoseWithPoint(pose, point):
    matrix = TransformToMatrix(PoseToTransform(pose))
    vector = PointToVector(point)
<<<<<<< HEAD
    composed = np.dot(matrix, vector)
=======
    composed = numpy.dot(matrix, vector)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    composed_point = PointFromVector(composed)
    return composed_point


def ComposeTransformWithPoint(transform, point):
    matrix = TransformToMatrix(transform)
    vector = PointToVector(point)
<<<<<<< HEAD
    composed = np.dot(matrix, vector)
=======
    composed = numpy.dot(matrix, vector)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    composed_point = PointFromVector(composed)
    return composed_point


def ComposeTransforms(transform1, transform2):
    tfmatrix1 = TransformToMatrix(transform1)
    tfmatrix2 = TransformToMatrix(transform2)
<<<<<<< HEAD
    composedmatrix = np.dot(tfmatrix1, tfmatrix2)
=======
    composedmatrix = dot(tfmatrix1, tfmatrix2)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    composed = TransformFromMatrix(composedmatrix)
    return composed


def ComposePoses(pose1, pose2):
    transform1 = PoseToTransform(pose1)
    transform2 = PoseToTransform(pose2)
    composed = PoseFromTransform(ComposeTransforms(transform1, transform2))
    return composed


def InvertTransform(old_transform):
    tfmatrix = TransformToMatrix(old_transform)
<<<<<<< HEAD
    xirtamft = np.linalg.inv(tfmatrix)
=======
    xirtamft = numpy.linalg.inv(tfmatrix)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    inverted = TransformFromMatrix(xirtamft)
    return inverted


def InvertPose(old_pose):
    old_transform = PoseToTransform(old_pose)
    inverted = InvertTransform(old_transform)
    posed = PoseFromTransform(inverted)
    return posed


def ComposeMatrices(matrix1, matrix2):
<<<<<<< HEAD
    return np.dot(matrix1, matrix2)


def InvertMatrix(old_matrix):
    return np.np.linalg.inv(old_matrix)
=======
    return dot(matrix1, matrix2)


def InvertMatrix(old_matrix):
    return numpy.linalg.inv(old_matrix)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default


def ComposeQuaternions(q1, q2):
    nq1 = NormalizeQuaternion(q1)
    nq2 = NormalizeQuaternion(q2)
<<<<<<< HEAD
    x = nq1[3] * nq2[0] + nq2[3] * nq1[0] + nq1[1] * nq2[2] - nq2[1] * nq1[2]
    y = nq1[3] * nq2[1] + nq2[3] * nq1[1] + nq2[0] * nq1[2] - nq1[0] * nq2[2]
    z = nq1[3] * nq2[2] + nq2[3] * nq1[2] + nq1[0] * nq2[1] - nq2[0] * nq1[1]
    w = nq1[3] * nq2[3] - nq1[0] * nq2[0] - nq1[1] * nq2[1] - nq1[2] * nq2[2]
=======
    x = nq1[3]*nq2[0] + nq2[3]*nq1[0] + nq1[1]*nq2[2] - nq2[1]*nq1[2]
    y = nq1[3]*nq2[1] + nq2[3]*nq1[1] + nq2[0]*nq1[2] - nq1[0]*nq2[2]
    z = nq1[3]*nq2[2] + nq2[3]*nq1[2] + nq1[0]*nq2[1] - nq2[0]*nq1[1]
    w = nq1[3]*nq2[3] - nq1[0]*nq2[0] - nq1[1]*nq2[1] - nq1[2]*nq2[2]
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    return NormalizeQuaternion([x, y, z, w])


def AxisFromQuaternion(q):
    nq = NormalizeQuaternion(q)
<<<<<<< HEAD
    a = np.arccos(nq[3]) * 2.0
    sina2 = sin(a * 0.5)
=======
    a = math.acos(nq[3]) * 2.0
    sina2 = math.sin(a * 0.5)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    if abs(sina2) > 0.000000001:
        i = nq[0] / sina2
        j = nq[1] / sina2
        k = nq[2] / sina2
        return [a, [i, j, k]]
    else:
        return [0.0, [0.0, 0.0, 1.0]]


def NormalizeVector3(axis):
<<<<<<< HEAD
    assert (len(axis) == 3)
    mag = np.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    assert (mag > 0.0)
=======
    assert(len(axis) == 3)
    mag = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    assert(mag > 0.0)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    return [axis[0] / mag, axis[1] / mag, axis[2] / mag]


def QuaternionFromAxisAngle(axis, angle):
    try:
        naxis = NormalizeVector3(axis)
<<<<<<< HEAD
        w = cos(angle * 0.5)
        x = sin(angle * 0.5) * naxis[0]
        y = sin(angle * 0.5) * naxis[1]
        z = sin(angle * 0.5) * naxis[2]
=======
        w = math.cos(angle * 0.5)
        x = math.sin(angle * 0.5) * naxis[0]
        y = math.sin(angle * 0.5) * naxis[1]
        z = math.sin(angle * 0.5) * naxis[2]
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
        return NormalizeQuaternion([x, y, z, w])
    except AssertionError:
        print("Vector normalize error, returning identity quaternion")
        return [0.0, 0.0, 0.0, 1.0]


def AngleBetweenQuaternions(q1, q2):
    nq1 = NormalizeQuaternion(q1)
    nq2 = NormalizeQuaternion(q2)
    dot_product = abs(nq1[0] * nq2[0] + nq1[1] * nq2[1] + nq1[2] * nq2[2] + nq1[3] * nq2[3])
    if dot_product < 0.9999:
<<<<<<< HEAD
        return np.arccos(2.0 * (dot_product ** 2) - 1.0)
=======
        return math.acos(2.0 * (dot_product ** 2) - 1.0)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    else:
        return 0.0


def AngleBetweenQuaternionsRos(q1, q2):
    nq1 = NormalizeQuaternionRos(q1)
    nq2 = NormalizeQuaternionRos(q2)
    dot_product = abs(nq1.x * nq2.x + nq1.y * nq2.y + nq1.z * nq2.z + nq1.z * nq2.w)
    if dot_product < 0.9999:
<<<<<<< HEAD
        return np.arccos(2.0 * (dot_product ** 2) - 1.0)
=======
        return math.acos(2.0 * (dot_product ** 2) - 1.0)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    else:
        return 0


'''Assist functions'''


def NormalizeQuaternion(q_raw):
<<<<<<< HEAD
    magnitude = np.sqrt(q_raw[0] ** 2 + q_raw[1] ** 2 + q_raw[2] ** 2 + q_raw[3] ** 2)
=======
    magnitude = math.sqrt(q_raw[0]**2 + q_raw[1]**2 + q_raw[2]**2 + q_raw[3]**2)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    x = q_raw[0] / magnitude
    y = q_raw[1] / magnitude
    z = q_raw[2] / magnitude
    w = q_raw[3] / magnitude
    return [x, y, z, w]


def NormalizeQuaternionRos(q_raw):
<<<<<<< HEAD
    magnitude = np.sqrt(q_raw.x ** 2 + q_raw.y ** 2 + q_raw.z ** 2 + q_raw.w ** 2)
=======
    magnitude = math.sqrt(q_raw.x**2 + q_raw.y**2 + q_raw.z**2 + q_raw.w**2)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    q_unit = Quaternion()
    q_unit.x = q_raw.x / magnitude
    q_unit.y = q_raw.y / magnitude
    q_unit.z = q_raw.z / magnitude
    q_unit.w = q_raw.w / magnitude
    return q_unit


'''Conversion functions'''


def PointToVector(point):
<<<<<<< HEAD
    return np.array([point.x, point.y, point.z, 1.0]).transpose()
=======
    return numpy.array([point.x, point.y, point.z, 1.0]).transpose()
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default


def PointFromVector(vector):
    new_point = Point()
    new_point.x = float(vector[0])
    new_point.y = float(vector[1])
    new_point.z = float(vector[2])
    return new_point


def PoseFromTransform(old_transform):
    posed = Pose()
    posed.position.x = old_transform.translation.x
    posed.position.y = old_transform.translation.y
    posed.position.z = old_transform.translation.z
    posed.orientation.x = old_transform.rotation.x
    posed.orientation.y = old_transform.rotation.y
    posed.orientation.z = old_transform.rotation.z
    posed.orientation.w = old_transform.rotation.w
    return posed


def PoseToTransform(old_pose):
    transformed = Transform()
    transformed.translation.x = old_pose.position.x
    transformed.translation.y = old_pose.position.y
    transformed.translation.z = old_pose.position.z
    transformed.rotation.x = old_pose.orientation.x
    transformed.rotation.y = old_pose.orientation.y
    transformed.rotation.z = old_pose.orientation.z
    transformed.rotation.w = old_pose.orientation.w
    return transformed


def PoseFromComponents(translation, quaternion):
    posed = Pose()
    posed.position.x = translation[0]
    posed.position.y = translation[1]
    posed.position.z = translation[2]
    posed.orientation.x = quaternion[0]
    posed.orientation.y = quaternion[1]
    posed.orientation.z = quaternion[2]
    posed.orientation.w = quaternion[3]
    return posed


def ComponentsFromPose(old_pose):
    translation = [old_pose.position.x, old_pose.position.y, old_pose.position.z]
    quaternion = [old_pose.orientation.x, old_pose.orientation.y, old_pose.orientation.z, old_pose.orientation.w]
    return [translation, quaternion]


def TransformFromComponents(translation, quaternion):
    transformed = Transform()
    transformed.translation.x = translation[0]
    transformed.translation.y = translation[1]
    transformed.translation.z = translation[2]
    transformed.rotation.x = quaternion[0]
    transformed.rotation.y = quaternion[1]
    transformed.rotation.z = quaternion[2]
    transformed.rotation.w = quaternion[3]
    return transformed


def ComponentsFromTransform(old_transform):
    translation = [old_transform.translation.x, old_transform.translation.y, old_transform.translation.z]
<<<<<<< HEAD
    quaternion = [old_transform.rotation.x, old_transform.rotation.y, old_transform.rotation.z,
                  old_transform.rotation.w]
=======
    quaternion = [old_transform.rotation.x, old_transform.rotation.y, old_transform.rotation.z, old_transform.rotation.w]
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    return [translation, quaternion]


def TransformToMatrix(old_transform):
    [translation, quaternion] = ComponentsFromTransform(old_transform)
    tfmatrix = BuildMatrix(translation, quaternion)
    return tfmatrix


def TransformFromMatrix(old_matrix):
    [translation, quaternion] = ExtractFromMatrix(old_matrix)
    transformed = TransformFromComponents(translation, quaternion)
    return transformed


def PoseToMatrix(old_pose):
    old_transform = PoseToTransform(old_pose)
    [translation, quaternion] = ComponentsFromTransform(old_transform)
    tfmatrix = BuildMatrix(translation, quaternion)
    return tfmatrix


def PoseFromMatrix(old_matrix):
    [translation, quaternion] = ExtractFromMatrix(old_matrix)
    transformed = TransformFromComponents(translation, quaternion)
    posed = PoseFromTransform(transformed)
    return posed


def ExtractRawFromMatrix(tm):
<<<<<<< HEAD
    rmat = np.array([[tm[0][0], tm[0][1], tm[0][2]],
                     [tm[1][0], tm[1][1], tm[1][2]],
                     [tm[2][0], tm[2][1], tm[2][2]]])
=======
    rmat = array([[tm[0][0], tm[0][1], tm[0][2]], [tm[1][0], tm[1][1], tm[1][2]], [tm[2][0], tm[2][1], tm[2][2]]])
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    tvec = [tm[0][3], tm[1][3], tm[2][3]]
    return [rmat, tvec]


def BuildRawMatrix(rm, tv):
<<<<<<< HEAD
    tfmatrix = np.array([[rm[0][0], rm[0][1], rm[0][2], tv[0]],
                         [rm[1][0], rm[1][1], rm[1][2], tv[1]],
                         [rm[2][0], rm[2][1], rm[2][2], tv[2]],
                         [0.0, 0.0, 0.0, 1.0]])
=======
    tfmatrix = array([[rm[0][0], rm[0][1], rm[0][2], tv[0]], [rm[1][0], rm[1][1], rm[1][2], tv[1]], [rm[2][0], rm[2][1], rm[2][2], tv[2]], [0.0, 0.0, 0.0, 1.0]])
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    return tfmatrix


def ExtractFromMatrix(old_matrix):
    quaternion = quaternion_from_matrix(old_matrix)
    translation = [old_matrix[0][3], old_matrix[1][3], old_matrix[2][3]]
    return [translation, quaternion]


def BuildMatrix(translation, quaternion):
    tfmatrix = quaternion_matrix(quaternion)
    tfmatrix[0][3] = translation[0]
    tfmatrix[1][3] = translation[1]
    tfmatrix[2][3] = translation[2]
    return tfmatrix


def BuildMatrixRos(translation, quaternion):
    quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    trans = [translation.x, translation.y, translation.z]
    return BuildMatrix(trans, quat)


''' Generation Functions '''

<<<<<<< HEAD

def BuildMatrixFromTransRot(trans, rot):
    transform = np.empty(shape=[4, 4])
=======
def BuildMatrixFromTransRot(trans, rot):
    transform = numpy.empty(shape=[4, 4])
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    transform[0:3, 0:3] = rot
    transform[0:3, 3] = trans
    transform[3, 0:3] = 0
    transform[3, 3] = 1
    return transform


def BuildMatrixFromDH(d, a, theta, alpha):
<<<<<<< HEAD
    # Do math here
    tfmatrix = np.array([
        [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), alpha * cos(theta)],
        [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), alpha * sin(theta)],
        [0.0, sin(alpha), cos(alpha), d], [0.0, 0.0, 0.0, 1.0]])
=======
    #Do math here
    tfmatrix = array([[math.cos(theta), -sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), alpha * math.cos(theta)], [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), alpha * math.sin(theta)], [0.0, math.sin(alpha), math.cos(alpha), d], [0.0, 0.0, 0.0, 1.0]])
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
    return tfmatrix


def ExtractFromDH(d, a, theta, alpha):
    tfmatrix = BuildMatrixFromDH(d, a, theta, alpha)
    [translation, quaternion] = ExtractFromMatrix(tfmatrix)
    return [translation, quaternion]


def TransformFromDH(d, a, theta, alpha):
    tfmatrix = BuildMatrixFromDH(d, a, theta, alpha)
    return TransformFromMatrix(tfmatrix)


def PoseFromDH(d, a, theta, alpha):
    tfmatrix = BuildMatrixFromDH(d, a, theta, alpha)
<<<<<<< HEAD
    return PoseFromMatrix(tfmatrix)


def spherical_to_vector3(r_phi_theta):
    """
    @param r_phi_theta: list-like object [r, phi, theta]
    @return: list of x, y, z
    """
    # https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
    r, phi, theta = r_phi_theta
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return [x, y, z]


def vector3_to_spherical(xyz):
    """
    @param xyz: list-like object [x, y, z]
    @return: list of r, phi, theta
    """
    # https://en.wikipedia.org/wiki/Spherical_coordinate_system#Cartesian_coordinates
    x, y, z = xyz
    r = np.sqrt(x ** 2 + y ** 2 + z ** 2)
    # phi is the angle about the z axis, where 0 is +x and pi/2 is +y
    phi = np.arctan2(y, x)
    # theta is inclination from z direction
    if r > 1e-6:
        theta = np.arccos(z / r)
    else:
        theta = 0
    return [r, phi, theta]
=======
    return PoseFromMatrix(tfmatrix)
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
