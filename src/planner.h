#ifndef PLANNER
#define PLANNER

#include <math.h>
#include <vector>

#define LANE_WIDTH 4
#define LANE_CENTER LANE_WIDTH / 2

#define TIME_INCREMENT 0.02 // second
#define MAX_ACC_SPECS 10
#define MPS_TO_MPH 2.237                                               // Meters per second to miles per hour
#define MAX_SPEED_INC TIME_INCREMENT *(MAX_ACC_SPECS - 4) * MPS_TO_MPH // (sec) * (m/s^2) * (mps to mph) = mph
#define MAX_SPEED_DEC TIME_INCREMENT *(MAX_ACC_SPECS - 3) * MPS_TO_MPH // (sec) * (m/s^2) * (mps to mph) = mph

#define SPLINE_INCREMENT_1 30
#define SPLINE_INCREMENT_2 60
#define SPLINE_INCREMENT_3 90

#define MAX_POINTS 50

#define SAFE_DIST 18

typedef struct _SensorFusion
{
  double id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
} SensorFusion;

vector<double> getInitialPoints(vector<double> &pts_x, vector<double> &pts_y,
                                vector<double> &previous_path_x, vector<double> &previous_path_y,
                                int prev_path_size,
                                double car_x, double car_y, double car_yaw)
{

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = 0;
    if (prev_path_size < 2)
    {
        ref_yaw = deg2rad(car_yaw);
        pts_x.push_back(car_x - cos(car_yaw));
        pts_y.push_back(car_y - sin(car_yaw));

        pts_x.push_back(car_x);
        pts_y.push_back(car_y);
    }
    else
    {
        ref_x = previous_path_x[prev_path_size - 1];
        ref_y = previous_path_y[prev_path_size - 1];

        double ref_x_prev = previous_path_x[prev_path_size - 2];
        double ref_y_prev = previous_path_y[prev_path_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        pts_x.push_back(ref_x_prev);
        pts_y.push_back(ref_y_prev);

        pts_x.push_back(ref_x);
        pts_y.push_back(ref_y);
    }

    return {ref_x, ref_y, ref_yaw};
}

void setNextWayPoints(vector<double> &pts_x, vector<double> &pts_y,
                      double car_s, double lane,
                      vector<double> &map_waypoints_s, vector<double> &map_waypoints_x, vector<double> &map_waypoints_y)
{
    vector<double> next_wp0 = getXY(car_s + SPLINE_INCREMENT_1, (LANE_CENTER + LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_s + SPLINE_INCREMENT_2, (LANE_CENTER + LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_s + SPLINE_INCREMENT_3, (LANE_CENTER + LANE_WIDTH * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

    pts_x.push_back(next_wp0[0]);
    pts_y.push_back(next_wp0[1]);

    pts_x.push_back(next_wp1[0]);
    pts_y.push_back(next_wp1[1]);

    pts_x.push_back(next_wp2[0]);
    pts_y.push_back(next_wp2[1]);
}

void shiftPlane(vector<double> &pts_x, vector<double> &pts_y,
                double ref_x, double ref_y, double ref_yaw)
{
    for (int i = 0; i < pts_x.size(); i++)
    {
        double new_x = pts_x[i] - ref_x;
        double new_y = pts_y[i] - ref_y;

        pts_x[i] = (new_x * cos(0 - ref_yaw)) - (new_y * sin(0 - ref_yaw));
        pts_y[i] = (new_x * sin(0 - ref_yaw)) + (new_y * cos(0 - ref_yaw));
    }
}

void pushFuturePoints(vector<double> &pts_x, vector<double> &pts_y,
                      vector<double> &next_x_vals, vector<double> &next_y_vals,
                      int prev_path_size, double ref_velocity,
                      double ref_x, double ref_y, double ref_yaw)
{

    tk::spline s;

    s.set_points(pts_x, pts_y);

    double target_x = 20;
    double target_y = s(target_x);

    double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
    double N = target_dist / (TIME_INCREMENT * ref_velocity / MPS_TO_MPH);
    double delta_x = target_x / N;

    double new_x = 0.0;
    double new_y = 0.0;

    for (int i = 0; i < (MAX_POINTS - prev_path_size); i++)
    {
        new_x = new_x + delta_x;
        new_y = s(new_x);

        // Shift Plane back
        double out_x = ref_x + (new_x * cos(ref_yaw)) - (new_y * sin(ref_yaw));
        double out_y = ref_y + (new_x * sin(ref_yaw)) + (new_y * cos(ref_yaw));

        next_x_vals.push_back(out_x);
        next_y_vals.push_back(out_y);
    }
}

#endif // PLANNER