import operator
import numpy as np
import gurobipy


# generate object array from dictionary with tuple key
def _gen_obj_arr(vars, shape):
    arr = np.empty(shape, dtype=np.object)
    for index in np.ndindex(shape):
        arr[index] = vars[index]
    return arr


def create_gurobi_arr(model, shape, name='', lb=-gurobipy.GRB.INFINITY):
    """
    Create gurobi variables as a numpy object array.
    :param model: instance of gurobipy.Model
    :param shape: tuple of numpy array shape
    :param name: name for gurobi variable
    :param lb: lower bound for the variable
    :return: numpy array of gurobi variables
    """
    vars = model.addVars(*shape, lb=lb, name=name)
    g_vars = _gen_obj_arr(vars, shape)
    return g_vars


def add_constraints(model, lhs, operation_type="<=", rhs=None, name=''):
    """
    Create element-wise constraint between gurobi variables and numpy array
    :param model: instance of gurobipy.Model
    :param lhs: A numpy object array of gurobi expressions
    :param operation_type: Comparison type, one of ["<=", "==", ">="]
    :param rhs: A numpy array of real values, same shape as lhs
    :param name: gurobi name for constraint objects
    :return: numpy array of gurobi constraints
    """
    python_operator = None
    if operation_type == "<=":
        python_operator = operator.le
    elif operation_type == "==":
        python_operator = operator.eq
    elif operation_type == ">=":
        python_operator = operator.ge
    else:
        raise RuntimeError("Unsupported operator {}".format(operation_type))

    if type(rhs) is not np.ndarray:
        raise RuntimeError("rhs is not numpy.ndarray!")

    if lhs.shape != rhs.shape:
        raise RuntimeError("lhs and rhs have different shape!")

    gen_temp_constrs = (python_operator(lhs[index], rhs[index]) for index in np.ndindex(lhs.shape))
    constr_dict = model.addConstrs(gen_temp_constrs, name=name)

    constr_arr = _gen_obj_arr(constr_dict, lhs.shape)
    return constr_arr


# generates numpy element-wise function
_get_value = np.vectorize(lambda var: var.X)


def get_value(g_arr):
    """
    Get optimization result from numpy array of gurobi variables
    :param g_arr: numpy array of gurobi variables
    :return: numpy array of float, same shape as g_arr
    """
    return _get_value(g_arr)
