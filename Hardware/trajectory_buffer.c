/* USART1整批轨迹接收与OPS连续跟踪；接收只存整数，校验/控制在50Hz任务。 */
#include "trajectory_buffer.h"
#include "main.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

typedef char TrajPointSizeCheck[(sizeof(TrajPoint_t) == 16U) ? 1 : -1];
static TrajPoint_t points[TRAJ_CAPACITY];
static volatile uint8_t state, cancel_req, run_req, resume_req;
static volatile uint16_t received, count, cursor;
static uint16_t verified, stop_index;
static uint32_t id, expected_crc, crc, upload_tick, run_tick, step_tick;
static uint32_t reply_generation, caps_generation;
static volatile uint8_t reply_pending, caps_pending;
static uint8_t error_code, initial_stop_gate;
static float margin, progress, best_progress, previous_x, previous_y, previous_yaw;
static float speed_measured, yaw_rate_measured, rotate_rate, path_speed;
static uint32_t pose_sequence, pose_tick, progress_tick, settled_ms;
TrajControlParams_t traj_control_params = {500.0f,6.0f,6.0f,6.0f,120.0f,60.0f,180.0f,60.0f,100.0f,600.0f,1000.0f,150.0f};

uint8_t Traj_SetControlParam(float *target,float value,float lower,float upper)
{
  if(!isfinite(value) || value<lower || value>upper) return 3U;
  if(Traj_Busy()) return 2U;
  *target=value;
  return 1U;
}

static float wrap(float a) { a = fmodf(a+180.0f,360.0f); if(a<0) a+=360.0f; return a-180.0f; }
static float minimum(float a,float b) { return a<b?a:b; }
static float clamp(float x,float a,float b) { return x<a?a:x>b?b:x; }
static void reply(void) { ++reply_generation; reply_pending=1U; }
static void fail(uint8_t code) { error_code=code; state=TRAJ_FAULT; run_req=resume_req=0U; reply(); }
static float px(uint16_t i) { return points[i].x10*0.1f; }
static float py(uint16_t i) { return points[i].y10*0.1f; }
static float ps(uint16_t i) { return points[i].s10*0.1f; }
static float yaw(uint16_t i) { return points[i].yaw100*0.01f; }
static float distance(float x,float y) { return sqrtf(x*x+y*y); }

/* 不接受空字段、尾部垃圾、溢出、非整数或符号错误。CRC字段为八位十六进制。 */
static uint8_t number(const char **text,int64_t *out,uint8_t hex)
{
  const char *p=*text; uint8_t negative=0U,digits=0U; uint64_t v=0U;
  if(!hex && *p=='-') { negative=1U; ++p; }
  while(*p && *p!=',') {
    uint8_t d;
    if(*p>='0' && *p<='9') d=(uint8_t)(*p-'0');
    else if(hex && *p>='A' && *p<='F') d=(uint8_t)(*p-'A'+10);
    else if(hex && *p>='a' && *p<='f') d=(uint8_t)(*p-'a'+10);
    else return 0U;
    v=v*(hex?16U:10U)+d;
    if(++digits>10U || v>4294967295ULL) return 0U;
    ++p;
  }
  if(!digits || (hex && digits!=8U)) return 0U;
  *out=negative?-(int64_t)v:(int64_t)v; *text=p; return 1U;
}
static uint8_t fields(const char *text,int64_t *v,uint8_t n,int8_t hex_index)
{
  uint8_t i;
  for(i=0U;i<n;++i) {
    if(!number(&text,&v[i],i==hex_index)) return 0U;
    if(i+1U<n) { if(*text!=',') return 0U; ++text; } else if(*text) return 0U;
  }
  return 1U;
}
static uint32_t crc_point(uint32_t value,const TrajPoint_t *p)
{
  uint8_t bytes[16],i,j; uint32_t v[3];
  v[0]=(uint32_t)p->x10; v[1]=(uint32_t)p->y10; v[2]=p->s10;
  for(i=0;i<12;++i) bytes[i]=(uint8_t)(v[i/4]>>(8*(i%4)));
  bytes[12]=(uint8_t)p->yaw100; bytes[13]=(uint8_t)((uint16_t)p->yaw100>>8);
  bytes[14]=(uint8_t)p->flags; bytes[15]=(uint8_t)(p->flags>>8);
  for(i=0;i<16;++i) { value^=bytes[i]; for(j=0;j<8;++j) value=(value>>1)^((value&1U)?0xEDB88320UL:0U); }
  return value;
}

void Traj_Init(void)
{
  const TrajControlParams_t defaults={500.0f,6.0f,6.0f,6.0f,120.0f,60.0f,180.0f,60.0f,100.0f,600.0f,1000.0f,150.0f};
  traj_control_params=defaults;
  state=TRAJ_IDLE; cancel_req=run_req=resume_req=0U;
  id=0U; count=received=cursor=0U; reply_pending=caps_pending=0U;
  reply_generation=caps_generation=0U; error_code=0U; rotate_rate=path_speed=0.0f;
}
uint8_t Traj_Busy(void) { return state>=TRAJ_RECEIVING && state<=TRAJ_WAITING; }
uint8_t Traj_OutputAllowed(void) { return !cancel_req && state==TRAJ_RUNNING; }
void Traj_Cancel(uint8_t reason)
{
  /* ISR和任务会重复确认取消；保留首个原因，不覆盖接收故障等诊断。 */
  if(Traj_Busy()) { if(!cancel_req) error_code=reason; cancel_req=1U; run_req=resume_req=0U; }
}

uint8_t Traj_CompleteStation(uint32_t batch_id,uint16_t point_index)
{
  uint8_t accepted=0U;uint32_t mask=__get_PRIMASK();__disable_irq();
  if(batch_id==id && point_index==cursor && state==TRAJ_WAITING && !cancel_req) {
    resume_req=1U;accepted=1U;
  }
  if(!mask)__enable_irq();
  return accepted;
}

uint8_t Traj_ParseLine(const char *line,uint32_t now,uint8_t upload_allowed)
{
  int64_t v[7]; TrajPoint_t p;
  if(strcmp(line,"TCAPS")==0) { ++caps_generation; caps_pending=1U; return 1U; }
  if(strncmp(line,"TBEGIN=",7)==0) {
    if(cancel_req || Traj_Busy() || !upload_allowed) { reply(); return 1U; }
    if(!fields(line+7,v,5,3) || v[0]<=0 || v[1]<2 || v[1]>TRAJ_CAPACITY ||
       v[2]<0 || v[4]<5 || v[4]>100) { fail(1); return 1U; }
    /* 地图版本在上传头校验；地图失效由主机取消，批次id绑定本次完整点表。 */
    id=(uint32_t)v[0]; count=(uint16_t)v[1]; expected_crc=(uint32_t)v[3];
    margin=(float)v[4]; crc=0xFFFFFFFFUL; received=cursor=verified=0U; progress=0;
    error_code=0U; upload_tick=now; state=TRAJ_RECEIVING; reply(); return 1U;
  }
  if(strncmp(line,"TPOINT=",7)==0) {
    if(cancel_req || state!=TRAJ_RECEIVING) { reply(); return 1U; }
    if(!fields(line+7,v,7,-1) || v[0]!=id || v[1]<0 || v[1]>=count ||
       v[2]<-100000 || v[2]>100000 || v[3]<-100000 || v[3]>100000 ||
       v[4]<-18000 || v[4]>=18000 || v[5]<0 || v[5]>10000000 || v[6]<0 || v[6]>15) {
      fail(2); return 1U;
    }
    p.x10=(int32_t)v[2]; p.y10=(int32_t)v[3]; p.yaw100=(int16_t)v[4];
    p.s10=(uint32_t)v[5]; p.flags=(uint16_t)v[6];
    if(v[1]<received) { if(memcmp(&p,&points[v[1]],sizeof(p))) fail(3); else reply(); return 1U; }
    if(v[1]!=received) { fail(3); return 1U; }
    points[received++]=p; crc=crc_point(crc,&p); upload_tick=now;
    if(received%TRAJ_WINDOW==0U || received==count) reply();
    return 1U;
  }
  if(strncmp(line,"TCOMMIT=",8)==0 || strncmp(line,"TRUN=",5)==0 ||
     strncmp(line,"TABORT=",7)==0 || strncmp(line,"TSTATUS=",8)==0) {
    const char *eq=strchr(line,'=');
    if(!fields(eq+1,v,1,-1) || v[0]!=id) { reply(); return 1U; }
    if(line[1]=='A') Traj_Cancel(0);
    else if(line[1]=='C') {
      if(state==TRAJ_RECEIVING && !cancel_req) {
        if(received!=count || (crc^0xFFFFFFFFUL)!=expected_crc) fail(4);
        else { state=TRAJ_VERIFYING; verified=0U; }
      }
    } else if(line[1]=='R') { if(state==TRAJ_READY && !cancel_req && upload_allowed) run_req=1U; }
    reply(); return 1U;
  }
  if(strncmp(line,"TRESUME=",8)==0) {
    if(fields(line+8,v,2,-1) && v[0]>0 && v[1]>=0 && v[1]<count)
      (void)Traj_CompleteStation((uint32_t)v[0],(uint16_t)v[1]);
    reply(); return 1U;
  }
  return 0U;
}

/* 前视距离可调，默认100mm；前馈取当前投影处几何增量，避免追前视弦切入障碍。 */
static void reference(float station,float out[3])
{
  uint16_t i=cursor;
  while(i<stop_index && ps(i+1)<station) ++i;
  if(i>=stop_index) { out[0]=px(stop_index);out[1]=py(stop_index);out[2]=yaw(stop_index); }
  else {
    float ds=ps(i+1)-ps(i), t=ds>0?clamp((station-ps(i))/ds,0,1):0;
    out[0]=px(i)+(px(i+1)-px(i))*t;out[1]=py(i)+(py(i+1)-py(i))*t;
    out[2]=yaw(i)+wrap(yaw(i+1)-yaw(i))*t;
  }
}
static void next_stop(void)
{
  path_speed=0.0f;
  stop_index=(uint16_t)(cursor+1U);
  while(stop_index+1U<count && !(points[stop_index].flags&TRAJ_STOP)) ++stop_index;
}
static uint8_t validate_point(uint16_t i)
{
  float ds,dxy,dyaw;
  if((points[i].flags&TRAJ_WAIT) && !(points[i].flags&TRAJ_STOP)) return 0;
  if((points[i].flags&TRAJ_ROTATE) && !(points[i].flags&TRAJ_STOP)) return 0;
  /* 点击路径允许先在真实起点停稳，再执行显式原地转头。 */
  if(i==0) return points[i].s10==0 && (points[i].flags&~TRAJ_STOP)==0;
  if(points[i].s10<points[i-1].s10) return 0;
  ds=ps(i)-ps(i-1); dxy=distance(px(i)-px(i-1),py(i)-py(i-1)); dyaw=fabsf(wrap(yaw(i)-yaw(i-1)));
  if(points[i].flags&TRAJ_ROTATE) return ds==0 && dxy<0.2f && (points[i-1].flags&TRAJ_STOP);
  if(ds<=0 || ds>20.2f || dxy>ds+0.2f || ds-dxy>0.3f || dyaw>5.1f) return 0;
  return 1U;
}

uint8_t Traj_Step(uint32_t now,const TrajPose_t *pose,uint8_t allowed,float velocity[3])
{
  uint16_t i; uint32_t dt=now-step_tick; uint8_t fresh;
  float travel,measured_dt=0;
  velocity[0]=velocity[1]=velocity[2]=0;
  if(cancel_req) { cancel_req=0; state=TRAJ_CANCELLED; reply(); return 2U; }
  if(!Traj_Busy()) return 0U;
  if(!allowed) { fail(5); return 2U; }
  if(state==TRAJ_RECEIVING && (uint32_t)(now-upload_tick)>3000U) { fail(6);return 2U; }
  if(state==TRAJ_VERIFYING) {
    uint16_t limit=verified+64U; if(limit>count) limit=count;
    for(;verified<limit;++verified) { if(!validate_point(verified)) { fail(7);return 2U; } }
    if(verified==count) {
      if(!(points[count-1].flags&TRAJ_STOP) || (points[count-1].flags&TRAJ_WAIT)) { fail(7);return 2U; }
      state=TRAJ_READY; reply();
    }
    return 0U;
  }
  if(state==TRAJ_RECEIVING) return 0U;
  if(!pose || !isfinite(pose->x) || !isfinite(pose->y) || !isfinite(pose->yaw) ||
     (uint32_t)(now-pose->pose_tick)>200U) { fail(8);return 2U; }
  if(state==TRAJ_READY) {
    if(!run_req) return 0U;
    run_req=0;
    if(distance(pose->x-px(0),pose->y-py(0))>5 || fabsf(wrap(pose->yaw-yaw(0)))>3) { fail(9);return 2U; }
    state=TRAJ_RUNNING; cursor=0; progress=0; next_stop();
    run_tick=step_tick=progress_tick=now; pose_tick=pose->pose_tick; pose_sequence=pose->sequence;
    previous_x=pose->x;previous_y=pose->y;previous_yaw=pose->yaw;
    best_progress=0;speed_measured=yaw_rate_measured=rotate_rate=path_speed=0;settled_ms=0;
    initial_stop_gate=(points[0].flags&TRAJ_STOP)!=0;
    reply();return 2U;
  }
  if((uint32_t)(now-run_tick)>180000U || dt>100U) { fail(10);return 2U; }
  step_tick=now;
  fresh=pose_sequence!=pose->sequence;
  travel=distance(pose->x-previous_x,pose->y-previous_y);
  if(fresh) {
    float angle_travel=fabsf(wrap(pose->yaw-previous_yaw));
    if(travel>50 || angle_travel>15) { fail(11);return 2U; }
    if((points[cursor+1U].flags&TRAJ_ROTATE) && angle_travel>0.2f) progress_tick=now;
    measured_dt=(uint32_t)(pose->pose_tick-pose_tick)*0.001f;
    if(measured_dt<=0 || measured_dt>0.2f) { fail(8);return 2U; }
    speed_measured=travel/measured_dt;
    yaw_rate_measured=fabsf(wrap(pose->yaw-previous_yaw))/measured_dt;
    pose_tick=pose->pose_tick;pose_sequence=pose->sequence;
    previous_x=pose->x;previous_y=pose->y;previous_yaw=pose->yaw;
  }
  if(state==TRAJ_WAITING) {
    if(distance(pose->x-px(cursor),pose->y-py(cursor))>5 || fabsf(wrap(pose->yaw-yaw(cursor)))>1) { fail(12);return 2U; }
    if(resume_req) {
      resume_req=0;
      if(speed_measured>5 || yaw_rate_measured>2) { reply();return 2U; }
      state=TRAJ_RUNNING;next_stop();progress=ps(cursor);progress_tick=now;best_progress=progress;settled_ms=0;reply();
    }
    return 0U;
  }
  if(initial_stop_gate) {
    float dx=px(0)-pose->x,dy=py(0)-pose->y,angle=wrap(yaw(0)-pose->yaw);
    float gap=distance(dx,dy);
    /* 起点STOP也必须真实停稳，不能因下一点ROTATE而跳过停靠门。 */
    if(gap+192.6f*fabsf(angle)*0.0174533f>margin-1.0f) { fail(13);return 2U; }
    velocity[0]=gap<0.5f?0:traj_control_params.kp_x*dx;velocity[1]=gap<0.5f?0:traj_control_params.kp_y*dy;
    {
      float mag=distance(velocity[0],velocity[1]);
      if(mag>traj_control_params.speed_mm_s) {
        velocity[0]*=traj_control_params.speed_mm_s/mag;
        velocity[1]*=traj_control_params.speed_mm_s/mag;
      }
    }
    velocity[2]=fabsf(angle)<0.2f?0:clamp(traj_control_params.kp_yaw*angle,-traj_control_params.yaw_rate_deg_s,traj_control_params.yaw_rate_deg_s);
    if(gap<5 && fabsf(angle)<1 && speed_measured<=5 && yaw_rate_measured<=2 && fresh)
      settled_ms+=(uint32_t)(measured_dt*1000+0.5f);
    else if(fresh) settled_ms=0;
    if(settled_ms>=200U) { initial_stop_gate=0;settled_ms=0;progress_tick=now;return 2U; }
    if((uint32_t)(now-progress_tick)>5000U) { fail(14);return 2U; }
    return 1U;
  }
  if(points[cursor+1U].flags&TRAJ_ROTATE) {
    float dx=px(cursor+1U)-pose->x,dy=py(cursor+1U)-pose->y;
    float gap=distance(dx,dy),angle=wrap(yaw(cursor+1U)-pose->yaw);
    float limit,target,change=traj_control_params.rotate_acc_deg_s2*dt*0.001f,mag;
    float hold_limit=minimum(traj_control_params.hold_speed_mm_s,traj_control_params.speed_mm_s);
    if(gap>margin-2) { fail(13);return 2U; }
    /* 原地转头仍需闭环保持车心；不能只转头而任由侧滑/安装偏心误差积累。 */
    velocity[0]=gap<0.5f?0.0f:traj_control_params.kp_x*dx;
    velocity[1]=gap<0.5f?0.0f:traj_control_params.kp_y*dy;
    mag=distance(velocity[0],velocity[1]);
    if(mag>hold_limit) { velocity[0]*=hold_limit/mag;velocity[1]*=hold_limit/mag; }
    /* 角速度和加速度使用轨迹RAM参数；临近目标按制动距离减速。
     * 偏离车心时降低转头速度，为位置保持留出响应时间；不放宽保护裕量。 */
    limit=minimum(minimum(traj_control_params.rotate_rate_deg_s,traj_control_params.yaw_rate_deg_s),0.8f*sqrtf(2.0f*traj_control_params.rotate_acc_deg_s2*fabsf(angle)));
    limit*=clamp((margin-2.0f-gap)/minimum(4.0f,margin-2.0f),0.0f,1.0f);
    target=fabsf(angle)<0.2f?0.0f:clamp(traj_control_params.kp_yaw*angle,-limit,limit);
    rotate_rate=clamp(target,rotate_rate-change,rotate_rate+change);
    velocity[2]=rotate_rate;
    if(distance(pose->x-px(cursor+1U),pose->y-py(cursor+1U))<5 && fabsf(angle)<1 &&
       speed_measured<=5 && yaw_rate_measured<=2 && fresh) settled_ms+=(uint32_t)(measured_dt*1000+0.5f);
    else if(fresh) settled_ms=0;
    if(settled_ms>=200U) {
      ++cursor;settled_ms=0;progress_tick=now;rotate_rate=0.0f;
      if(cursor+1U==count) state=TRAJ_DONE;
      else if(points[cursor].flags&TRAJ_WAIT) state=TRAJ_WAITING;
      else next_stop();
      reply();return 2U;
    }
    if((uint32_t)(now-progress_tick)>5000U) { fail(14);return 2U; }
    return 1U;
  }
  rotate_rate=0.0f;
  {
    float high=minimum(ps(stop_index),progress+travel+20),low=progress,best=1e20f,station=progress;
    float here[3],ahead[3],ref[3],speed=traj_control_params.speed_mm_s,heading_error,mag,remaining;
    float horizon,deviation,limit=margin-1.0f,output_limit=traj_control_params.speed_mm_s;
    for(i=cursor;i<stop_index && ps(i)<=high;++i) {
      float ds=ps(i+1)-ps(i),dx=px(i+1)-px(i),dy=py(i+1)-py(i),norm=dx*dx+dy*dy;
      float a=ds>0?clamp((low-ps(i))/ds,0,1):0,b=ds>0?clamp((high-ps(i))/ds,0,1):0;
      float t=norm>0?clamp(((pose->x-px(i))*dx+(pose->y-py(i))*dy)/norm,a,b):0;
      float d=distance(px(i)+t*dx-pose->x,py(i)+t*dy-pose->y);
      if(d<best-0.0001f) { best=d;station=ps(i)+t*ds; }
    }
    progress=station;
    while(cursor+1U<stop_index && ps(cursor+1U)<=progress) ++cursor;
    reference(progress,here);reference(minimum(ps(stop_index),progress+traj_control_params.lookahead_mm),ref);
    heading_error=wrap(here[2]-pose->yaw);
    /* 规划完整车体已有margin；实际中心和航向偏差必须留在该裕量内。 */
    deviation=best+192.6f*fabsf(heading_error)*0.0174533f;
    if(deviation>limit) { fail(13);return 2U; }
    /* 扫描完整制动距离，允许直线高速，并在未来圆弧入口前提前减速。
     * arc标记即使车头不变也限速；航向变化另受可用角速度限制。 */
    horizon=progress+traj_control_params.speed_mm_s*traj_control_params.speed_mm_s/
      (2.0f*traj_control_params.decel_mm_s2)+traj_control_params.lookahead_mm;
    for(i=cursor;i<stop_index && ps(i)<horizon;++i) {
      float ds=ps(i+1)-ps(i),angle=fabsf(wrap(yaw(i+1)-yaw(i)));
      float local=traj_control_params.speed_mm_s,gap=ps(i)-progress;
      if(gap<0) gap=0;
      if(points[i+1].flags&TRAJ_ARC) local=minimum(local,traj_control_params.arc_speed_mm_s);
      if(ds>0 && angle>0.001f) local=minimum(local,ds*traj_control_params.yaw_rate_deg_s*0.8f/angle);
      if(i==cursor) output_limit=minimum(output_limit,local);
      speed=minimum(speed,sqrtf(local*local+2.0f*traj_control_params.decel_mm_s2*gap));
    }
    remaining=ps(stop_index)-progress;
    if(remaining<traj_control_params.lookahead_mm) speed=minimum(speed,4*distance(ref[0]-pose->x,ref[1]-pose->y));
    speed=minimum(speed,sqrtf(2.0f*traj_control_params.decel_mm_s2*remaining));
    /* 越接近偏差门限越降低路径前馈，保留位置/航向纠偏与原超限停车。 */
    if(deviation>0.4f*limit) speed*=clamp((0.85f*limit-deviation)/(0.45f*limit),0,1);
    if(dt==0) return 0;
    path_speed=clamp(speed,path_speed-traj_control_params.decel_mm_s2*dt*0.001f,
                           path_speed+traj_control_params.accel_mm_s2*dt*0.001f);
    /* 轨迹限速/偏差保护优先于斜坡，不能因旧速度越过圆弧或停车点。 */
    path_speed=minimum(path_speed,speed);
    reference(minimum(ps(stop_index),progress+path_speed*dt*0.001f),ahead);
    if(dt==0) return 0;
    velocity[0]=(ahead[0]-here[0])*1000/dt+traj_control_params.kp_x*(here[0]-pose->x);
    velocity[1]=(ahead[1]-here[1])*1000/dt+traj_control_params.kp_y*(here[1]-pose->y);
    velocity[2]=clamp(wrap(ahead[2]-here[2])*1000/dt+traj_control_params.kp_yaw*heading_error,-traj_control_params.yaw_rate_deg_s,traj_control_params.yaw_rate_deg_s);
    mag=distance(velocity[0],velocity[1]);if(mag>output_limit) { velocity[0]*=output_limit/mag;velocity[1]*=output_limit/mag; }
    if(remaining<5 && distance(pose->x-px(stop_index),pose->y-py(stop_index))<5 &&
       fabsf(wrap(pose->yaw-yaw(stop_index)))<1 && speed_measured<=5 && yaw_rate_measured<=2 && fresh)
      settled_ms+=(uint32_t)(measured_dt*1000+0.5f);
    else if(fresh) settled_ms=0;
    if(remaining<0.5f && distance(pose->x-px(stop_index),pose->y-py(stop_index))<0.5f) velocity[0]=velocity[1]=0;
    if(settled_ms>=200U) {
      cursor=stop_index;progress=ps(cursor);settled_ms=0;
      if(cursor+1U==count) state=TRAJ_DONE;
      else if(points[cursor].flags&TRAJ_WAIT) state=TRAJ_WAITING;
      else next_stop();
      best_progress=progress;progress_tick=now;reply();return 2U;
    }
    if(progress>best_progress+1) { best_progress=progress;progress_tick=now; }
    if((uint32_t)(now-progress_tick)>5000U) { fail(14);return 2U; }
  }
  if(now%200U<dt) reply();
  return 1U;
}

uint16_t Traj_PeekReply(char *out,uint16_t capacity,TrajReply_t *token)
{
  int n;uint32_t mask=__get_PRIMASK(),local_id,local_progress;
  uint16_t local_received,local_cursor;uint8_t local_state,local_error;
  __disable_irq();
  if(caps_pending) { token->kind=1;token->generation=caps_generation; }
  else if(reply_pending) { token->kind=2;token->generation=reply_generation; }
  else { if(!mask)__enable_irq();return 0; }
  local_id=id;local_progress=(uint32_t)(progress*10+0.5f);local_received=received;
  local_cursor=cursor;local_state=state;local_error=error_code;
  if(!mask)__enable_irq();
  if(token->kind==1) n=snprintf(out,capacity,"TCAPS 1 %u %u\r\n",(unsigned)TRAJ_CAPACITY,(unsigned)TRAJ_WINDOW);
  else n=snprintf(out,capacity,"TSTAT %lu %u %u %u %lu %u\r\n",(unsigned long)local_id,
    (unsigned)local_state,(unsigned)local_received,(unsigned)local_cursor,(unsigned long)local_progress,(unsigned)local_error);
  return n>0 && n<capacity?(uint16_t)n:0;
}
void Traj_ReplySent(const TrajReply_t *token)
{
  uint32_t mask=__get_PRIMASK();__disable_irq();
  if(token->kind==1 && token->generation==caps_generation) caps_pending=0;
  if(token->kind==2 && token->generation==reply_generation) reply_pending=0;
  if(!mask)__enable_irq();
}
