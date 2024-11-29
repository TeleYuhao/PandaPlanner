import casadi as ca

def VehicleDynamic():
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    v = ca.SX.sym('v')
    yaw = ca.SX.sym('yaw')
    delta = ca.SX.sym('delta')
    a = ca.SX.sym('a')

    state = ca.vertcat(x, y, yaw, v)
    control = ca.vertcat(delta,a)
    rhs = ca.vertcat(x + v * ca.cos(yaw),
                            y + v * ca.sin(yaw ),yaw,v)
    DynamicFunc = ca.Function("DynamicFunc",[state],[rhs])
    return DynamicFunc