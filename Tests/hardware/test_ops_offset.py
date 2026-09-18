"""运行真实 OPS 坐标换算和补偿命令解析，使用统一坐标系合成数据。

OPS 原始帧固定映射到统一坐标：
    X(车左) = -raw_y
    Y(车头) = -raw_x

默认安装：OPS 在车体中心左 60mm、后 50mm，即统一坐标 (60,-50) mm。
"""
from pathlib import Path
import re
import subprocess
import tempfile


root = Path(__file__).resolve().parents[2]
ops = (root / "Hardware/ops.c").read_text(encoding="utf-8")
debug = (root / "Hardware/debug_usart.c").read_text(encoding="utf-8")


def function(source, name):
    point = re.search(r"^(?:static )?(?:uint8_t|void) " + name + r"\(", source, re.M).start()
    start = source.rfind("\n", 0, point) + 1
    brace = source.index("{", point)
    level, end = 1, brace + 1
    while level:
        level += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


prelude = r'''
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
static struct {
  struct {float x,y,z;} frame;
  unsigned valid_count;
  uint8_t pose_valid;
  float origin_x,origin_y;
  uint8_t zero_enabled;
} s_ops;
static float s_mount_x_mm=60.0f, s_mount_y_mm=-50.0f;
static float s_reference_yaw, s_origin_yaw;
static uint8_t s_new_flag;
static unsigned mask;
static unsigned __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __enable_irq(void) {mask=0;}
static void near(float a,float b) {
  if (fabsf(a-b)>=0.00001f) printf("near mismatch: got %.8f expected %.8f\n",a,b);
  assert(fabsf(a-b)<0.00001f);
}

/* 合成物理位姿 (tx,ty)：中心 X=左、Y=前，单位 m。
 * 固定逆映射为 raw_x=-Y、raw_y=-X，因此：
 *   中心原始坐标 = (-ty,-tx)
 *   左60/后50 的安装偏移在原始帧为 (+0.05,-0.06)
 * 传感器原始坐标 = 中心原始坐标 + R(angle)*r_raw。 */
static void pose(float angle,float tx,float ty) {
  float c=cosf(angle), s=sinf(angle);
  s_ops.frame.x=-ty + 0.05f*c + 0.06f*s;
  s_ops.frame.y=-tx + 0.05f*s - 0.06f*c;
  s_ops.frame.z=angle;
  s_ops.valid_count=1;
  s_ops.pose_valid=1;
  s_new_flag=1;
}
'''


checks = r'''
int main(void) {
  float x,y,z,v[2];
  unsigned i;
  const float pi2=1.5707963268f, pi=3.1415926536f;
  const char *bad[]={"-501,60","-50,501","nan,60","-50,60x","-50,60,0","-50,","1e2,60","--50,60"};

  assert(Debug_ParseOffset("60,-50",v));
  near(v[0],60.0f); near(v[1],-50.0f);
  for(i=0;i<sizeof(bad)/sizeof(bad[0]);++i) assert(!Debug_ParseOffset(bad[i],v));
  assert(!OPS_SetMountOffset(NAN,60));
  assert(!OPS_SetMountOffset(60,INFINITY));
  assert(!OPS_SetMountOffset(501,-50));

  /* 绝对接口仍然只做原始帧到统一坐标的映射，不叠加补偿。 */
  {
    const float angles[]={0,pi2,pi,2*pi,-pi2};
    for(i=0;i<5;++i) {
      pose(angles[i],0,0);
      OPS_CopyPosition(&x,&y,&z,1);
      near(x,-s_ops.frame.y);
      near(y,-s_ops.frame.x);
      near(z,angles[i]);
    }
  }

  /* 置零后，纯平移与“平移+旋转”都返回同一个物理中心位移。 */
  pose(0,0,0);
  OPS_ZeroCoordinates();
  pose(pi2,0.2f,-0.1f);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0.2f); near(y,-0.1f); near(z,pi2);
  pose(pi2,0,0);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0); near(y,0);
  /* 实车验收方向：向前只增加 Y；向左只增加 X。 */
  pose(0,0,0.1f);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0); near(y,0.1f);
  pose(0,0.1f,0);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0.1f); near(y,0);

  /* 非零参考航向下置零，继续原地旋转仍保持中心为零。 */
  pose(pi2,0,0);
  OPS_ZeroCoordinates();
  pose(pi,0,0);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0); near(y,0); near(z,pi2);

  /* 配置错误为0时，旋转会留下安装半径对应的残差；配置正确后消失。 */
  assert(OPS_SetMountOffset(0,0));
  pose(pi + pi2,0,0);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0.11f); near(y,0.01f);
  assert(hypotf(x,y) > 0.110f);
  assert(OPS_SetMountOffset(60,-50));
  pose(pi,0,0);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0); near(y,0);

  /* 临界区保护：原中断状态为1时必须保持。 */
  mask=1;
  assert(OPS_SetMountOffset(60,-50));
  assert(mask==1);
  OPS_CopyPosition(&x,&y,&z,0);
  near(x,0); near(y,0);
  assert(mask==1);
  mask=0;

  /* 无有效位姿时仍保存配置，但不建立新的置零基准。 */
  s_ops.valid_count=0;
  assert(OPS_SetMountOffset(0,0));
  near(s_mount_x_mm,0.0f);
  near(s_mount_y_mm,0.0f);

  puts("Unified OPS coordinate/offset/zero/raw/validation tests passed");
  return 0;
}
'''


names = [
    "OPS_MapRawToUnified",
    "OPS_MapUnifiedToRaw",
    "OPS_CopyPosition",
    "OPS_ZeroCoordinates",
    "OPS_ClearZero",
    "OPS_SetMountOffset",
]
code = (prelude
        + "\n".join(function(ops, name) for name in names)
        + function(debug, "Debug_ParseFloat")
        + function(debug, "Debug_ParseOffset")
        + checks)

with tempfile.TemporaryDirectory(prefix="ilhc_ops_offset_") as directory:
    folder = Path(directory)
    src, exe = folder / "test.c", folder / "test.exe"
    src.write_text(code, encoding="utf-8")
    subprocess.run(["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror", str(src),
                    "-lm", "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
