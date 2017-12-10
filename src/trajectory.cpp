#include "trajectory.h"
#include <math.h>
#include "spline.h"

using namespace std;
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }

trajectory::trajectory() {}
vector<double> trajectory::getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
        int prev_wp = -1;

        while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
        {
                prev_wp++;
        }

        int wp2 = (prev_wp+1)%maps_x.size();

        double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
        // the x,y,s along the segment
        double seg_s = (s-maps_s[prev_wp]);

        double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
        double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

        double perp_heading = heading-pi()/2;

        double x = seg_x + d*cos(perp_heading);
        double y = seg_y + d*sin(perp_heading);

        return {x,y};

}
void trajectory::get_trajectory(std::vector<double>& next_x_vals, std::vector<double>& next_y_vals, vehicle our_car, int prev_size, vector<double> previous_path_x, vector<double> previous_path_y, int lane, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, double ref_vel)
{
vector<double> ptsx;

vector<double> ptsy;

double ref_x = our_car.X;
double ref_y = our_car.Y;
double ref_yaw = deg2rad(our_car.Yaw);

if(prev_size < 2)
{
        double prev_car_x = our_car.X - cos(our_car.Yaw);
        double prev_car_y = our_car.Y - sin(our_car.Yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(our_car.X);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(our_car.Y);
}
else
{
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
}


vector<double> next_wp0 = getXY(our_car.S+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(our_car.S+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(our_car.S+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

for(int i = 0; i < ptsx.size(); i++)
{
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
}
tk::spline s;
s.set_points(ptsx, ptsy);
for(int i = 0; i < previous_path_x.size(); i++)
{
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
}
double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
double x_add_on = 0;

double dist_inc = 0.3;
    for(int i = 0; i < 50-previous_path_x.size(); i++)
    {
        double N = (target_dist/(.02*ref_vel/2.24));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
        y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

        x_point += ref_x;
        y_point += ref_y;


          next_x_vals.push_back(x_point);
          next_y_vals.push_back(y_point);

    }
}
// Transform from Frenet s,d coordinates to Cartesian x,y

