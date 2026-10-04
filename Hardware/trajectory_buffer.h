#ifndef TRAJECTORY_BUFFER_H
#define TRAJECTORY_BUFFER_H
#include <stdint.h>

/* 整批点在运行前缓存于SRAM；每点16字节，绝不写Flash。 */
#define TRAJ_CAPACITY 4096U
#define TRAJ_WINDOW 3U
#define TRAJ_STOP 1U
#define TRAJ_WAIT 2U
#define TRAJ_ROTATE 4U
#define TRAJ_ARC 8U
#define TRAJ_IDLE 0U
#define TRAJ_RECEIVING 1U
#define TRAJ_VERIFYING 2U
#define TRAJ_READY 3U
#define TRAJ_RUNNING 4U
#define TRAJ_WAITING 5U
#define TRAJ_DONE 6U
#define TRAJ_CANCELLED 7U
#define TRAJ_FAULT 8U

/* 坐标为OPS零点坐标：X左、Y前；yaw零点朝前、逆时针正。 */
typedef struct {
  int32_t x10, y10;
  uint32_t s10;
  int16_t yaw100;
  uint16_t flags;
} TrajPoint_t;
typedef struct {
  float x, y, yaw;
  uint32_t sequence, pose_tick;
} TrajPose_t;
typedef struct { uint32_t generation; uint8_t kind; } TrajReply_t;

/* 连续轨迹RAM调参，世界X/Y为mm，航向为deg；掉电恢复默认值。 */
typedef struct {
  float speed_mm_s, kp_x, kp_y, kp_yaw, yaw_rate_deg_s;
  float rotate_rate_deg_s, rotate_acc_deg_s2, hold_speed_mm_s, lookahead_mm;
  float accel_mm_s2, decel_mm_s2, arc_speed_mm_s;
} TrajControlParams_t;
extern TrajControlParams_t traj_control_params;
/* 1成功、2轨迹忙、3非有限值或越界；必须通过此接口写参数。 */
uint8_t Traj_SetControlParam(float *target, float value, float lower, float upper);
void Traj_Init(void);
uint8_t Traj_ParseLine(const char *line, uint32_t now, uint8_t upload_allowed);
void Traj_Cancel(uint8_t reason);
uint8_t Traj_Busy(void);
uint8_t Traj_OutputAllowed(void);
/* 后续真实塔吊任务完成后调用：只释放匹配批次/站点的WAIT，不能恢复已取消路径。 */
uint8_t Traj_CompleteStation(uint32_t batch_id, uint16_t point_index);
/* 任务计算OPS世界速度mm/s、角速度deg/s；返回1输出速度，2必须停车，0无输出。 */
uint8_t Traj_Step(uint32_t now, const TrajPose_t *pose, uint8_t allowed, float velocity[3]);
uint16_t Traj_PeekReply(char *out, uint16_t capacity, TrajReply_t *token);
void Traj_ReplySent(const TrajReply_t *token);
#endif
