import operator
import numpy as np
import gurobipy


def _gen_obj_arr(vars, shape):
    arr = np.empty(shape, dtype=np.object)
    for index in np.ndindex(shape):
        arr[index] = vars[index]
    return arr


def create_gurobi_arr(model, shape, name='', lb=-gurobipy.GRB.INFINITY):
    vars = model.addVars(*shape, lb=lb, name=name)
    g_vars = _gen_obj_arr(vars, shape)
    return g_vars


def add_constraints(model, lhs, operation_type="<=", rhs=None, name=''):
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


_get_value = np.vectorize(lambda var: var.X)


def get_value(g_arr):
    return _get_value(g_arr)
