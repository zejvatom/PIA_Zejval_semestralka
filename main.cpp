#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include "Vehicle.h"


using namespace std;

double g = 9.81; //m/s^2

//car parameters

double m; //kg - mass of the car
double vmax; //m/s
double mi; //tyre friction coefficient
double h; //mm - height of the COG
double L; //mm - wheelbase
double T; //mm - track of the car
double xCOG; //1 - x COG position from front axle
double yCOG; //1 - y COG position from right side
double P; //kW - power of the engine
double WD; //mm - wheel diameter
double dr; //1 - drive ratio
double rpm; //rpm - engine rpm
double t_shift; //s - time to shift gear
int gear; //number of gears

double W, WR, WF, TF, CF, DF;    //Forces

//load data from file
void load_data(string filename) {
    ifstream file;
    file.open(filename, ios::in);
    if (!file) throw runtime_error("Unable to load from file: " + filename);
    string line;
    int i = 0;
    while (getline(file, line)) {
        i++;
        switch (i) {
            case 1:
                m = stod(line);
                break;
            case 2:
                vmax = stod(line);
                break;
            case 3:
                mi = stod(line);
                break;
            case 4:
                h = stod(line);
                break;
            case 5:
                L = stod(line);
                break;
            case 6:
                T = stod(line);
                break;
            case 7:
                xCOG = stod(line);
                break;
            case 8:
                yCOG = stod(line);
                break;
            case 9:
                P = stod(line);
                break;
            case 10:
                WD = stod(line);
                break;
            case 11:
                dr = stod(line);
                break;
            case 12:
                rpm = stod(line);
                break;
            case 13:
                t_shift = stod(line);
                break;
            case 14:
                gear = stoi(line);
                break;
            default:
                break;
        }
    }
    file.close();

}
Vehicle* inicialize(string input) {
    load_data(input);
    Vehicle* vehicle;
    vehicle = new Vehicle(m, vmax, mi, h, L, T, xCOG, yCOG, P, WD, dr, rpm, t_shift, gear);

    return vehicle;
}

void Forces(){
    //Forces
    W = m * g; //N - weight of the car
    WR = W * (L * xCOG) / L; //N - weight on the rear axle
    WF = W - WR; //N - weight on the front axle
    TF = WR * mi / (1 - (h * mi / L)); //N - traction force
    CF = W * mi; //N - cornering force
    DF = 60 * P * 1000 / (M_PI * rpm * dr); //N - drag force
    if (DF > TF) {
        DF = TF;
    } else {
        DF = DF;
    }
}

//SKIDPAD_____________________________________________________________________________________

double r, s, tw, r_race, v_skid, t_skid;
double mio_max, mio_min, mio_step, mio, miof, CFo, v_skido, t_skido, v_reduce;
int kmax;
double mio_values[1000];
double tskid_values[1000];
double tskid_min;

void SkidPad() {
    //parameters of the track
    r = 9.125; //m - centerline radius of the skidpad
    s = 2 * M_PI * r; //m - length of the skidpad
    tw = 3; //m - width of the skidpad
    r_race = r - tw / 2 + T / 1000 * yCOG + 0.05; //m - radius of raceline

    //calculations
    v_skid = sqrt(CF * r_race / m); //m/s - cornering speed during skidpad
    t_skid = s / v_skid; //s - time to complete skidpad
}

void SkidPad_opt() {
    //Different tyre compounds - NOT WORKING
    mio_max = 1.8;
    mio_min = 1.4;
    mio_step = 0.1;
    miof = 0;
    v_reduce = 0.5; //m/s - fake variable to reduce the speed
    kmax = (mio_max - mio_min) / mio_step;

    mio_values[kmax] = {0};
    tskid_values[kmax] = {0};
    tskid_min = 99999999.9;

    for (int k = 0; k < kmax; k++) {
        mio = mio_min + k * mio_step;
        CFo = W * mio;
        v_skido = sqrt(CFo * r_race / m) - k * v_reduce;
        t_skido = s / v_skido;

        mio_values[k] = mio;
        tskid_values[k] = t_skido;
    }

    for (int kk = 0; kk < kmax; kk++) {
        if (tskid_values[kk] < tskid_min) {
            tskid_min = tskid_values[kk];
            miof = mio_values[kk];
        }
    }
}

//ACCELERATION_________________________________________________________________________________
double s_acc, a, shift_acc, vmaxt, vmax_total, t_acc, s_vmax, t_vmax, s_diff, t2, t_vmax_total, t_vmax_totalo;

void Acceleration(){
    //parameters of the track
    s_acc = 75; //m

    //calculations
    a = DF / m; //m/s^2
    shift_acc = gear - 1;

    //calculate max speed
    vmaxt = sqrt(2 * a * s_acc); //m/s

    if (vmaxt < vmax) {
        t_vmax = vmax / a; //s - time to reach max speed
        s_vmax = a * (t_vmax + shift_acc * t_shift) * (t_vmax + shift_acc * t_shift) / 2; //m - distance to reach max speed
        s_diff = s_acc - s_vmax; //m - difference between s_acc and s_vmax
        t2 = s_diff / vmax; //s - time of s_diff with max speed
        t_acc = t_vmax + t2 + shift_acc * t_shift; //s - total time to complete acceleration
        vmax_total = vmax; //m/s - total max speed
        t_vmax_total = t_vmax; //s - time to reach max speed
    } else {
        t_acc = sqrt(2 * s_acc / a) + shift_acc * t_shift; //s - time to reach max speed
        vmax_total = vmaxt;
        t_vmax_total = t_acc; //s - time to reach max speed
    }

}

double xCOG_min, xCOG_max, xCOG_step, m_ballast;
double mo, xCOGo, xCOGof, WRo, WFo, TFo, Fl, Flo, ao, vmaxto, t_vmaxo, s_vmaxo, s_diffo, t2o, vmax_totalo, t_acco;
int jmax;
double m_min;
double xCOGo_values[1000];
double ta_values[1000];
double ta_min;

void Acceleration_opt(){
    xCOG_min = 0.4;
    xCOG_max = 0.6;
    xCOG_step = 0.01;
    m_ballast = 2.0;
    Fl = 11.6; //N - force aero lift, fake force

    jmax = (xCOG_max - xCOG_min) / xCOG_step + 2; //number of iterations
    m_min = m - (xCOG - xCOG_min) * 100 * m_ballast;

    xCOGo_values[jmax] = {0};
    ta_values[jmax] = {0};
    ta_min = 99999999.9;

    for (int j = 0; j < jmax; j++) {
        xCOGo = xCOG_min + j * xCOG_step;
        Flo = Fl * j;
        mo = m_min + j * m_ballast;  //kg - mass of the car with ballast
        WRo = W * (L * xCOGo) / L - Flo; //N - weight on the rear axle
        WFo = W - WRo; //N - weight on the front axle
        TFo = WRo * mio / (1 - (h * mio / L)); //N - traction force
        ao = TFo / mo; //m/s^2
        vmaxto = sqrt(2 * ao * s_acc); //m/s
        if (DF>TFo){
            DF=TFo;
        } else{

        }


        if (vmaxto < vmax) {
            t_vmaxo = vmax / ao; //s - time to reach max speed
            s_vmaxo = ao * (t_vmaxo + shift_acc * t_shift) * (t_vmaxo + shift_acc * t_shift) /
                      2; //m - distance to reach max speed
            s_diffo = s_acc - s_vmaxo; //m - difference between s_acc and s_vmax
            t2o = s_diffo / vmax; //s - time of s_diff with max speed
            t_acco = t_vmaxo + t2o + shift_acc * t_shift; //s - total optimized time
            vmax_totalo = vmax;
            t_vmax_totalo = t_vmaxo; //s - time to reach max speed
        } else {
            t_acco = sqrt(2 * s_acc / ao) + shift_acc * t_shift; //s
            vmax_totalo = vmaxto;
            t_vmax_totalo = t_acco; //s - time to reach max speed
        }
        ta_values[j] = t_acco;
        xCOGo_values[j] = xCOGo;

    }

    for (int jj = 0; jj < jmax; jj++) {
        if (ta_values[jj] < ta_min) {
            ta_min = ta_values[jj];
            xCOGof = xCOGo_values[jj];
        }
    }
}

int main() {
    Vehicle *vehicle;
    vehicle = inicialize("D:\\School\\AVSI\\PIA\\Lap_time\\Lap_time\\parameters.txt");

    Forces();

    SkidPad();

    SkidPad_opt();

    Acceleration();

    Acceleration_opt();

    //RESULTS

    ofstream outputs("outputs.txt");
    outputs << "RESULTS" << endl << endl;

    outputs << "Car parameters:" << endl;
    outputs << "- Mass: " << m << " kg" << endl;
    outputs << "- Wheelbase: " << L << " mm" << endl;
    outputs << "- Track: " << T << " mm" << endl;
    outputs << "- Height of the COG: " << h << " mm" << endl;
    outputs << "- xCOG: " << xCOG << "from the front axle/" << endl;
    outputs << "- yCOG: " << yCOG << " from the ground" << endl;
    outputs << "- Max speed: " << vmax << " m/s" << endl;
    outputs << "- Tyre friction coefficient: " << mi << endl;
    outputs << "- Power: " << P << " W" << endl;
    outputs << "- Gear ratio: " << gear << endl;
    outputs << "- RPM for max torque: " << rpm << " rpm" << endl;
    outputs << "- Wheel diameter: " << WD << " mm" << endl;
    outputs << "- Gearbox: " << gear << " gears" << endl;
    outputs << "- Shifting time: " << t_shift << " s" << endl << endl << endl;

    outputs << "SKIDPAD EVENT" << endl << endl;
    outputs << "- Avg speed: " << v_skid << " m/s (" << v_skid * 3.6 << " km/h)" << endl;
    outputs << "- Time of one lap: " << t_skid << " s" << endl << endl << endl;

    outputs << "ACCELERATION EVENT" << endl << endl;
    outputs << "- Acceleration: " << a << " m/s^2" << endl;
    outputs << "- Time: " << t_acc << " s" << endl;
    outputs << "- Max speed: " << vmax_total << " m/s (" << vmax_total * 3.6 << " km/h)" << endl;
    outputs << "- Time to reach max speed: " << t_vmax_total << " s" << endl << endl << endl;

    outputs << "OPTIMIZATION" << endl << endl;
    outputs << "- Friction coefficient of optimal tyre compound: " << mio << endl;
    outputs << "- Time of skidpad: " << tskid_min << " s" << endl << endl;
    outputs << "- xCOG: " << xCOGof << " from the front axle" << endl;
    outputs << "- Time of acceleration: " << ta_min << " s" << endl << endl << endl;

    outputs << "FORCES" << endl << endl;
    outputs << "- Weight of the car: " << W << " N" << endl;
    outputs << "- Weight on the rear axle: " << WR << " N" << endl;
    outputs << "- Weight on the front axle: " << WF << " N" << endl;
    outputs << "- Cornering force: " << CF << " N" << endl;
    outputs << "- Traction force: " << TF << " N" << endl;
    outputs << "- Driven force: " << DF << " N" << endl << endl << endl;

    if (DF>TF){
        outputs << "WARNING: Driven force is higher than traction force." << endl;
    } else {
        outputs << "The car will be able to accelerate." << endl;
    }

    cout << "Calcutaion of the skidpad and acceleration completed. Results (including optimization) are written in file outputs.txt" << endl;

}



