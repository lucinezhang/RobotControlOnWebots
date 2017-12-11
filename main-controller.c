//只掉一个块
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/connector.h>
#include <webots/receiver.h>
#include <webots/compass.h>
#include <webots/emitter.h>
#include <webots/gps.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 16
#define TARGET_POINTS_SIZE 2
#define DISTANCE_TOLERANCE 0.2
#define ANGLE_TOLERANCE 0.1
#define MAX_SPEED 40.0

#define LEFT 0
#define RIGHT 1

#define NB_DIST_SENS 8
#define PS_RIGHT_00 0
#define PS_RIGHT_45 1
#define PS_RIGHT_90 2
#define PS_RIGHT_REAR 3
#define PS_LEFT_REAR 4
#define PS_LEFT_90 5
#define PS_LEFT_45 6
#define PS_LEFT_00 7

#define OAM_OBST_THRESHOLD 500
#define OAM_K_PS_90 0.04
#define OAM_K_PS_45 0.18
#define OAM_K_PS_00 0.24
#define OAM_K_MAX_DELTAS 60
#define MAX_OA_TIME 100

enum XYZAComponents {X, Y, Z, ALPHA};

typedef enum { WALK_TOGETHER, PAUSE_TOGETHER, ONE_MOVE, FIND, ADD} states;
typedef struct _Vector
{
    double u;
    double v;
} Vector;

static WbDeviceTag gps;
static WbDeviceTag compass;
static WbDeviceTag receiver;
static WbDeviceTag emitter;
static WbDeviceTag rear_connector, front_connector;
static WbDeviceTag ps[NB_DIST_SENS];
static char ps_names[2][8][7] = {
   {"ds1_1r", "ds1_2r", "ds1_3r", "ds1_4r", "ds1_4l", "ds1_3l", "ds1_2l", "ds1_1l"},
   {"ds2_1r", "ds2_2r", "ds2_3r", "ds2_4r", "ds2_4l", "ds2_3l", "ds2_2l", "ds2_1l"}
};
static int ps_value[NB_DIST_SENS]={0,0,0,0,0,0,0,0};
static int Activation[2]={0,0};
static int oam_active = 0;
static int oam_side = LEFT;
static int oam_time = 0;

static int walk_time = 500;//初始时大蛇的行走时间
static int pause_together = 128;
static double left_speed = 0.0, right_speed = 0.0;
static double rotation_speed = 10.0;
static int id = -1;
static int in_position = 0;//0——初始时还未运动到位；1——运动到位并静止；2——被接走
static int all_in_position = 0;//大蛇是否到model_1处
int flag = 0;
static int pause_before_find = 3600;
static Vector targets[TARGET_POINTS_SIZE] = {
  {-2, -6},
  {-1.9,-5.9}
};

static double modulus_double(double a, double m)
{
  int div_i = (int) (a/m);
  double div_d = (double) div_i;
  double r = a - div_d * m;
  if (r<0.0)
    r += m;
  return r;
}

// ||v||
static double norm(const Vector *v)
{
  return sqrt(v->u*v->u + v->v*v->v);
}

// v = v/||v||
static void normalize(Vector *v)
{
  double n = norm(v);
  v->u /= n;
  v->v /= n;
}

// v = v1-v2
static void minus(Vector *v, const Vector *v1, const Vector *v2)
{
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

// compute the angle between two vectors
// return value: [0, 2Pi]
static double angle(const Vector *v1, const Vector *v2)
{
  return modulus_double(atan2(v2->v, v2->u) - atan2(v1->v, v1->u) , 2.0*M_PI);
}

void ObstacleExistence(void)
{
  int i;
  int max_ds_value = 0;
  for (i=0; i<NB_DIST_SENS; i++)
    ps_value[i] = wb_distance_sensor_get_value(ps[i]); 

  max_ds_value = 0;
    
  for (i = PS_RIGHT_00; i <= PS_RIGHT_45; i++) 
  {
    if (max_ds_value < ps_value[i]) 
      max_ds_value = ps_value[i];
    Activation[RIGHT] += ps_value[i];
  }
  for (i = PS_LEFT_45; i <= PS_LEFT_00; i++) 
  {
    if (max_ds_value < ps_value[i]) 
      max_ds_value = ps_value[i];
    Activation[LEFT] += ps_value[i];
  }
  if (max_ds_value > OAM_OBST_THRESHOLD) 
  {
    oam_active = true;
    if(oam_time == 0)
      oam_time = MAX_OA_TIME;
  }
  else
    oam_active = false;
}

void ObstacleAvoidance(void)
{
  int DeltaS=0;

  // if (oam_side == NO_SIDE) // check for side of obstacle only when not already detected
  if (Activation[RIGHT] > Activation[LEFT]) 
    oam_side = RIGHT;
  else 
    oam_side = LEFT;

  // Forward speed
  left_speed  = MAX_SPEED;
  right_speed = MAX_SPEED;

  // Go away from obstacle
  if(oam_active == true)
  {
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT)
    {
      DeltaS -= (int) (OAM_K_PS_90 * ps_value[PS_LEFT_90]); //(((ps_value[PS_LEFT_90]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_90]-PS_OFFSET)));
      DeltaS -= (int) (OAM_K_PS_45 * ps_value[PS_LEFT_45]); //(((ps_value[PS_LEFT_45]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_45]-PS_OFFSET)));
      DeltaS -= (int) (OAM_K_PS_00 * ps_value[PS_LEFT_00]); //(((ps_value[PS_LEFT_00]-PS_OFFSET)<0)?0:(ps_value[PS_LEFT_00]-PS_OFFSET)));
    }
    else // oam_side == RIGHT
    {
      DeltaS += (int) (OAM_K_PS_90 * ps_value[PS_RIGHT_90]);  //(((ps_value[PS_RIGHT_90]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_90]-PS_OFFSET)));
      DeltaS += (int) (OAM_K_PS_45 * ps_value[PS_RIGHT_45]);  //(((ps_value[PS_RIGHT_45]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_45]-PS_OFFSET)));
      DeltaS += (int) (OAM_K_PS_00 * ps_value[PS_RIGHT_00]);  //(((ps_value[PS_RIGHT_00]-PS_OFFSET)<0)?0:(ps_value[PS_RIGHT_00]-PS_OFFSET)));
    }
    if (DeltaS > OAM_K_MAX_DELTAS) 
      DeltaS = OAM_K_MAX_DELTAS;
    if (DeltaS < -OAM_K_MAX_DELTAS) 
      DeltaS = -OAM_K_MAX_DELTAS;
  }
  // Set speeds
  left_speed -= DeltaS;
  right_speed += DeltaS;
}

// autopilot
// pass trough the predefined target positions
static void run_autopilot(const Vector *t)
{
  if(t == NULL)
  {
    left_speed = MAX_SPEED;
    right_speed = MAX_SPEED;
    return;
  }

  // prepare the speed array
  left_speed = 0.0;
  right_speed = 0.0;

  // read gps position and compass values
  const double *pos3D = wb_gps_get_values(gps);
  const double *north3D = wb_compass_get_values(compass);

  // compute the 2D position of the robo and its orientation
  Vector pos   = { pos3D[X], pos3D[Z] };
  Vector north = { north3D[X], north3D[Z] };
  Vector front = { north.v, north.u };

  // compute the direction and the distance to the target
  Vector dir;
  minus(&dir, t, &pos);
  double distance = norm(&dir);
  normalize(&dir);

  // compute the target angle
  double beta = angle( &front, &dir ) - M_PI;

  // a target position has been reached
  if (distance < DISTANCE_TOLERANCE)
  {
    if(id == 0)
      in_position = 1;
    else if(id == 1)
      all_in_position = 1;
  }
  else
  {
    // big turn
    if (beta > ANGLE_TOLERANCE)
    {
      left_speed  =  MAX_SPEED;
      right_speed = -MAX_SPEED;
    } 
    else if (beta < -ANGLE_TOLERANCE)
    {
      left_speed  = -MAX_SPEED;
      right_speed =  MAX_SPEED;
    }
    // go forward with small rectifications
    else
    {
      left_speed  = MAX_SPEED - M_PI + beta;
      right_speed = MAX_SPEED - M_PI - beta;
    }
  }
}

void snake_run(int head_id, double head_left_speed, double head_right_speed)
{
  double v1 = MAX_SPEED;
  double v2 = MAX_SPEED;
  
  if(id == head_id) 
  {
    v1 = head_left_speed;
    v2 = head_right_speed;
    
    double message[2];
    message[0] = v1;
    message[1] = v2;
    wb_emitter_send(emitter, (void *)message, 2*sizeof(double));
  }
  else if(id > head_id) 
  {
    if(wb_receiver_get_queue_length(receiver)==0) 
    {
      v1 = MAX_SPEED;
      v2 = MAX_SPEED;
    }
    else 
    {
      const void *buffer0 = wb_receiver_get_data(receiver);
      double *buffer = (double *)buffer0;
      v1 = buffer[0];
      v2 = buffer[1];
      wb_receiver_next_packet(receiver);
    }
    double message[2];
    message[0] = v1;
    message[1] = v2;
    wb_emitter_send(emitter, (void *)message, 2*sizeof(double));
  }
  left_speed = v1;
  right_speed = v2;
}


int main(int argc, char *argv[])
{
  // initialize webots communication
  wb_robot_init();

  states state;

  const char *name = wb_robot_get_name();
  id = atoi(name + 7) - 1;
  char c_id = (char)id+'0'+1;
  char receiver_name[] = "receiver_1";
  receiver_name[9] = c_id;
  receiver_name[10] = '\0';
  receiver = wb_robot_get_device(receiver_name);
  wb_receiver_enable(receiver,TIME_STEP);
  char emitter_name[] = "emitter_1";
  emitter_name[8] = c_id;
  emitter_name[9] = '\0';
  emitter = wb_robot_get_device(emitter_name);

  // get gps tag and enable
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // get compass tag and enable
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

  //get front and rear connector
  rear_connector  = wb_robot_get_device("rear_connector");
  front_connector = wb_robot_get_device("front_connector");

  //get distance sensor and enable
  if(id == 0 || id == 1)
  {
    for (int i=0; i < NB_DIST_SENS ; i++) 
    {
      ps[i] = wb_robot_get_device(ps_names[id][i]);
      wb_distance_sensor_enable(ps[i], TIME_STEP);
    }
  }
  
  if(id == 0)
  {
    wb_connector_enable_presence(rear_connector, TIME_STEP);
  }
  if(id == 1)
  {
    wb_connector_enable_presence(front_connector, TIME_STEP);
  }

  // start forward motion
  //wb_differential_wheels_set_speed(MAX_SPEED,MAX_SPEED);

  state = WALK_TOGETHER;
  // main loop
  while (wb_robot_step(TIME_STEP) != -1)
  {
    left_speed = 0.0;
    right_speed = 0.0;

    switch(state)
    {
      case WALK_TOGETHER:       //整体运动一段时间
        if(walk_time > 0)
        {
          walk_time--;

          if(id == 0)
          {
            ObstacleExistence();
            if(oam_time > 0)
            {
              ObstacleAvoidance();
              oam_time --;
            }
            else
            {
              if(oam_active == false)
                run_autopilot(NULL);
              else
                ObstacleAvoidance();
            }
          }
          snake_run(0, left_speed, right_speed);
        }
        else if(walk_time == 0)
        {
          state = PAUSE_TOGETHER;
        }
        break;

      case PAUSE_TOGETHER:       //整体停一段时间、model_1脱落
        if(pause_together > 0)
        {
          pause_together--;
          left_speed = 0.0;
          right_speed = 0.0;       
        }
        else if(pause_together == 0)
        {
          //model_1脱落
          if(id == 0)
            wb_connector_unlock(rear_connector);
          else if(id == 1)
            wb_connector_unlock(front_connector);

          state = ONE_MOVE;
        }
        break;

      case ONE_MOVE:       //model_1运动至目的位置，其余静止
        if(in_position == 0)
        {
          if(id == 0)
          {
            ObstacleExistence();
            if(oam_time > 0)
            {
              ObstacleAvoidance();
              oam_time --;
            }
            else
            {
              if(oam_active == false)
                run_autopilot(&targets[0]);
              else
                ObstacleAvoidance();
            }       
          }
          else
          {
            if(pause_before_find > 0)
            {
              left_speed = 0.0;
              right_speed = 0.0;
              pause_before_find--;
            }
            else
              in_position = 1;       
          }
        }
        else if(in_position == 1)             //model_1到达
          state = FIND;

        break;

      case FIND:       //model_1 已到达目的位置,旋转等待,其余的来找他
        if(id == 0)
        {
          left_speed = rotation_speed;
          right_speed = -rotation_speed;
          wb_connector_lock(rear_connector);
          if(wb_connector_get_presence(rear_connector) == 1)
            state = ADD;
        }
        else
        {
            if(id == 1)
            {
              ObstacleExistence();
              if(oam_time > 0)
              {
                ObstacleAvoidance();
                oam_time --;
              }
              else
              {
                if(oam_active == false)
                  run_autopilot(&targets[1]);
                else
                  ObstacleAvoidance();
              }
            }
            snake_run(1, left_speed, right_speed);
            
            if(id == 1)
            {
              wb_connector_lock(front_connector);
              if(wb_connector_get_presence(front_connector) == 1)
                flag = 1;
            }
            if(flag)
              state = ADD;
        }
        break;
        
      case ADD:
        walk_time = 1000;
        state = WALK_TOGETHER;
        left_speed = 0.0;
        right_speed = 0.0;
        break;
    }
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }
 
  wb_robot_cleanup();

  return 0;
}
