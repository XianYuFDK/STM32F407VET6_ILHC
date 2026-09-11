"""模拟GPIO HAL，验证真实初始化顺序、默认电平及CubeMX分配一致性。"""
from pathlib import Path
import tempfile, subprocess, re
root=Path(__file__).resolve().parents[2]
header=(root/"Core/Inc/main.h").read_text(encoding="utf-8")
macros="\n".join(line for line in header.splitlines() if re.match(r"#define (VM_EN|CAMERA_LIGHT_EN|START_KEY[12]|CAMERA_[TR]X|GRIPPER_PWM)_",line))
config={}
for line in (root/"STM32F407VET6_ILHC.ioc").read_text(encoding="utf-8").splitlines():
    if "=" in line:
        k,v=line.split("=",1)
        assert k not in config, k
        config[k]=v
assert int(config["Mcu.PinsNb"])==len([k for k in config if re.fullmatch(r"Mcu.Pin[0-9]+",k)])
for pin,label in [("PD0","VM_EN"),("PD1","CAMERA_LIGHT_EN"),("PD2","START_KEY1"),("PD3","START_KEY2")]:
    assert config[pin+".GPIO_Label"]==label
    assert pin in config.values()
    if pin in ("PD0","PD1"):
        assert config[pin+".Signal"]=="GPIO_Output" and config[pin+".PinState"]=="GPIO_PIN_RESET"
    else:
        assert config[pin+".Signal"]=="GPIO_Input" and config[pin+".GPIO_PuPd"]=="GPIO_PULLUP"
pre=r'''
#include <stdint.h>
#include <assert.h>
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_9 512U
#define GPIO_PIN_10 1024U
#define GPIO_PIN_11 2048U
#define GPIOD ((void *)4)
#define GPIOB ((void *)2)
#define GPIOE ((void *)5)
#define GPIO_PIN_RESET 0
#define GPIO_MODE_OUTPUT_PP 1
#define GPIO_MODE_INPUT 0
#define GPIO_NOPULL 0
#define GPIO_PULLUP 1
#define GPIO_SPEED_FREQ_LOW 0
#define __HAL_RCC_GPIOH_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() (clock_on=1)
typedef struct {uint32_t Pin,Mode,Pull,Speed;} GPIO_InitTypeDef;
static int clock_on,step;
static void HAL_GPIO_WritePin(void *p,uint32_t pins,int value){assert(clock_on&&step==0&&p==GPIOD&&pins==3&&value==0);step++;}
static void HAL_GPIO_Init(void *p,GPIO_InitTypeDef *s){
 assert(p==GPIOD);
 if(step==1){assert(s->Pin==3&&s->Mode==GPIO_MODE_OUTPUT_PP&&s->Pull==GPIO_NOPULL&&s->Speed==GPIO_SPEED_FREQ_LOW);}
 else {assert(step==2&&s->Pin==12&&s->Mode==GPIO_MODE_INPUT&&s->Pull==GPIO_PULLUP);}
 step++;
}
'''
with tempfile.TemporaryDirectory(prefix="ilhc_gpio_") as d:
    p=Path(d)
    (p/"gpio.h").write_text(pre+macros,encoding="utf-8")
    body=(root/"Core/Src/gpio.c").read_text(encoding="utf-8")
    (p/"test.c").write_text(body+"\nint main(void){MX_GPIO_Init();assert(step==3);return 0;}\n",encoding="utf-8")
    subprocess.run(["gcc","-std=c99","-Wall","-Wextra","-Werror","-I",str(p),str(p/"test.c"),"-o",str(p/"test.exe")],check=True)
    subprocess.run([str(p/"test.exe")],check=True)
print("Board GPIO and CubeMX configuration tests passed")
