"""Independent illustrative wrench equilibrium; never a flight controller."""
import numpy as np
from scipy.optimize import brentq
from .trajectory2d.dynamics import G


def trim(force,control,speed,slope_deg):
    a=force.profile.airframe;gamma=np.deg2rad(slope_deg);qS=.5*a.air_density_kg_m3*speed**2*a.wing_area_m2
    def evaluate(alpha):
        elevator=control.pitch_stiffness*(alpha-control.alpha_trim_rad)/control.pitch_control
        cl=force.assumptions.lift_slope_per_rad*(alpha-force.assumptions.zero_lift_angle_rad)+elevator
        drag=qS*(a.zero_lift_drag_coefficient+a.induced_drag_factor*cl*cl)
        axial=drag+a.mass_kg*G*np.sin(gamma)
        thrust=max(0.,axial)/np.cos(alpha);brake=max(0.,-axial)
        normal=qS*cl+thrust*np.sin(alpha)-a.mass_kg*G*np.cos(gamma)
        return normal,elevator,cl,thrust,brake
    if control.pitch_control==0:raise ValueError('trim requires nonzero pitch authority')
    alpha=brentq(lambda alpha:evaluate(alpha)[0],-.35,.35,xtol=1e-14)
    residual,elevator,cl,thrust,brake=evaluate(alpha)
    if abs(elevator)>1 or abs(cl)>a.max_lift_coefficient or thrust>a.max_thrust_n or brake>a.max_airbrake_drag_n:
        raise ValueError('requested trim exceeds lift, control, thrust or brake authority')
    result=dict(speed_m_s=speed,slope_deg=slope_deg,alpha_rad=alpha,elevator_fraction=elevator,
                lift_coefficient=cl,thrust_n=thrust,brake_n=brake,
                brake_fraction=brake/a.max_airbrake_drag_n if a.max_airbrake_drag_n else 0.,normal_residual_n=residual)
    return {key:float(value) for key,value in result.items()}
