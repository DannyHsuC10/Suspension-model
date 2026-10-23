import numpy as np

class GeometryModel:

    def __init__(
        self,
        v_geo,
        roll_center,
        anti
    ):

        self.v_geo = v_geo
        self.roll_center = roll_center
        self.anti = anti

    def rc_force(self,state):

        Fy = state.Fy
        v_geo = self.v_geo

        h_rc_f,h_rc_r = self.roll_center.get_height(state)
        
        F_rc = np.zeros(4)

        # front
        Fy_front = Fy[0]+Fy[1]

        dF_front = (Fy_front * h_rc_f/ v_geo.tf)

        F_rc[0] += dF_front
        F_rc[1] -= dF_front

        # rear
        Fy_rear = Fy[2]+Fy[3]

        dF_rear = (Fy_rear * h_rc_r / v_geo.tr)

        F_rc[2] += dF_rear
        F_rc[3] -= dF_rear

        M_geo_fx = Fy_front*h_rc_f
        M_geo_rx =  Fy_rear*h_rc_r

        return F_rc,M_geo_fx,M_geo_rx

    def anti_force(self, state):

        Fx = state.Fx
        v_geo = self.v_geo

        dive_front,lift_front,squat_rear,lift_rear=self.anti.get_rate(state)


        F_anti = np.zeros(4)

        # Front axle
        for i in [0,1]:

            fx = Fx[i]

            if fx < 0:
                # braking
                rate = dive_front
            else:
                # driving
                rate = lift_front

            F_anti[i] = -(fx* rate* v_geo.h_cg/ v_geo.lf)

        # Rear axle
        for i in [2,3]:

            fx = Fx[i]

            if fx > 0:
                # driving
                rate = squat_rear
            else:
                # braking
                rate = lift_rear

            F_anti[i] = -(fx* rate* v_geo.h_cg/ v_geo.lr)

        # pitch moment
        F_front = F_anti[0] + F_anti[1]
        F_rear = F_anti[2] + F_anti[3]


        M_geoy = -(F_front * v_geo.lf - F_rear * v_geo.lr)


        return F_anti, M_geoy

    def force(self,state):

        F_rc,M_geo_fx,M_geo_rx = self.rc_force(state)
        F_anti,M_geoy = self.anti_force(state)

        M_geo = np.array([M_geo_fx,M_geo_rx,M_geoy,0.0])

        F_geo = F_rc+F_anti

        return F_geo,M_geo

