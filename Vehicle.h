//
// Created by zejva on 13.12.2022.
//

#ifndef OPTIMUM_TRACK_2_VEHICLE_H
#define OPTIMUM_TRACK_2_VEHICLE_H


class Vehicle {
public:
    double m; //kg - mass of the car
    double vmax; //m/s
    double mi; //tyre friction coefficient
    double h; //mm - height of the COG
    double L; //mm - wheelbase
    double T; //mm - track of the car
    double xCOG; //1 - x COG position from front axle
    double yCOG; //1 - y COG position from front axle
    double P; //kW - power of the engine
    double WD; //mm - wheel diameter
    double dr; //1 - drive ratio
    double rpm; //rpm - engine rpm
    double t_shift; //s - time to shift gear
    int gear; //number of gears
    Vehicle(double m_in, double vmax_in, double mi_in, double h_in, double L_in, double T_in, double xCOG_in, double yCOG_in, double P_in, double WD_in, double dr_in, double rpm_in, double t_shift_in, int gear_in) {
        m = m_in;
        vmax = vmax_in;
        mi = mi_in;
        h = h_in;
        L = L_in;
        T = T_in;
        xCOG = xCOG_in;
        yCOG = yCOG_in;
        P = P_in;
        WD = WD_in;
        dr = dr_in;
        rpm = rpm_in;
        t_shift = t_shift_in;
        gear = gear_in;
    }

};


#endif //OPTIMUM_TRACK_2_VEHICLE_H
