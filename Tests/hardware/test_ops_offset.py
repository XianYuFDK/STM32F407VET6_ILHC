"""运行真实OPS坐标换算和补偿命令解析，使用合成几何数据，不连接硬件。"""
from pathlib import Path
import re
import subprocess
import tempfile
r = Path(__file__).resolve().parents[2]
ops = (r / "Hardware/ops.c").read_text(encoding="utf-8")
debug = (r / "Hardware/debug_usart.c").read_text(encoding="utf-8")
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
static struct {struct {float x,y,z;} frame; unsigned valid_count; float origin_x,origin_y; uint8_t zero_enabled;} s_ops;
static float s_mount_x_mm=-50, s_mount_y_mm=60, s_reference_yaw, s_origin_yaw;
static uint8_t s_new_flag;
static unsigned mask;
static unsigned __get_PRIMASK(void) {return mask;}
static void __disable_irq(void) {mask=1;}
static void __enable_irq(void) {mask=0;}
static void near(float a,float b) {assert(fabsf(a-b)<0.00001f);}
/* 合成偏心点轨迹，中心平移(tx,ty)，偏移实际为后50/左60mm。 */
static void pose(float angle,float tx,float ty) {
 float dc=cosf(angle)-cosf(s_reference_yaw), ds=sinf(angle)-sinf(s_reference_yaw);
 s_ops.frame.x=tx+dc*(-0.05f)-ds*0.06f;
 s_ops.frame.y=ty+ds*(-0.05f)+dc*0.06f;
 s_ops.frame.z=angle; s_ops.valid_count=1; s_new_flag=1;
}
'''
checks = r'''
int main(void) {
 float x,y,z,v[2]; unsigned i;
 float angles[]={0,1.5707963268f,3.1415926536f,6.2831853072f,-1.5707963268f};
 const char *bad[]={"-501,60","-50,501","nan,60","-50,60x","-50,60,0","-50,","1e2,60","--50,60"};
 assert(Debug_ParseOffset("-50.5,+60.2",v)); near(v[0],-50.5f); near(v[1],60.2f);
 for(i=0;i<sizeof(bad)/sizeof(bad[0]);++i) assert(!Debug_ParseOffset(bad[i],v));
 assert(!OPS_SetMountOffset(NAN,60)); assert(!OPS_SetMountOffset(-50,INFINITY));
 for(i=0;i<5;++i) {
  pose(angles[i],0,0); OPS_CopyPosition(&x,&y,&z,0); near(x,0);near(y,0);
  OPS_CopyPosition(&x,&y,&z,1);near(x,s_ops.frame.x);near(y,s_ops.frame.y);
 }
 pose(0,0,0); OPS_ZeroCoordinates();
 pose(1.5707963268f,0.2f,-0.1f); OPS_CopyPosition(&x,&y,&z,0);near(x,-0.2f);near(y,0.1f);
 /* 非零角度处ZERO，继续原地转动中心仍为零。 */
 pose(1.5707963268f,0,0); OPS_ZeroCoordinates();
 pose(3.1415926536f,0,0); OPS_CopyPosition(&x,&y,&z,0);near(x,0);near(y,0);
 assert(OPS_SetMountOffset(0,0));
 pose(1.5707963268f,0,0); OPS_CopyPosition(&x,&y,&z,0);assert(hypotf(x,y)>0.1f);
 mask=1; assert(OPS_SetMountOffset(-50,60)); assert(mask==1);
 OPS_CopyPosition(&x,&y,&z,0);near(x,0);near(y,0);assert(mask==1);
 mask=0; OPS_ClearZero(); s_reference_yaw=0.7f;
 pose(2.0f,0.2f,-0.1f);OPS_CopyPosition(&x,&y,&z,0);near(x,0.2f);near(y,-0.1f);
 s_ops.valid_count=0; assert(OPS_SetMountOffset(-50,60)); assert(mask==0);
 puts("OPS offset: rotation / translation / ZERO / raw / nonzero reference / validation passed");
 return 0;
}
'''
names = ["OPS_CopyPosition", "OPS_ZeroCoordinates", "OPS_ClearZero", "OPS_SetMountOffset"]
code=prelude+"\n".join(function(ops,n) for n in names)+function(debug,"Debug_ParseFloat")+function(debug,"Debug_ParseOffset")+checks
with tempfile.TemporaryDirectory(prefix="ilhc_ops_offset_") as directory:
    folder=Path(directory)
    src,exe=folder/"test.c",folder/"test.exe"
    src.write_text(code,encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror",str(src),"-lm","-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
