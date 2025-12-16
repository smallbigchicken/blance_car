#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <signal.h>

// -------------- 硬件GPIO定义 --------------
#define KEY3_GPIO     3    // KEY3对应GPIO编号
#define LED1_GPIO     2    // LED1对应GPIO编号
#define LED2_GPIO     38   // LED2对应GPIO编号（若不可用，替换为38（LED4））
// -------------- 时间参数定义 --------------
#define DEBOUNCE_MS   10    // 消抖延时（10ms）
#define LONG_PRESS_MS 1000 // 长按判断时间（1秒）
#define LED1_INTERVAL 1000 // LED1交替间隔（1秒）
#define LED2_FAST_MS  200  // LED2快速闪烁间隔（200ms）
#define LED2_SLOW_MS  1000 // LED2慢速闪烁间隔（1秒）
#define LIFT_OFF_MS   5000 // 抬起后熄灭计时（5秒）

// -------------- GPIO操作函数 --------------
// 导出GPIO（内核→用户空间）
int gpio_export(int gpio) {
    char path[50] = {0};
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        perror("Failed to open export file");
        return -1;
    }
    char gpio_str[10] = {0};
    snprintf(gpio_str, sizeof(gpio_str), "%d", gpio);
    if (write(fd, gpio_str, strlen(gpio_str)) < 0) {
        perror("Failed to export GPIO");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

// 设置GPIO方向（in/out）
int gpio_set_dir(int gpio, const char *dir) {
    char path[50] = {0};
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open direction file");
        return -1;
    }
    if (write(fd, dir, strlen(dir)) < 0) {
        perror("Failed to set GPIO direction");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

// 读取GPIO值（用于按键）
int gpio_get_value(int gpio) {
    char path[50] = {0};
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_RDONLY);
    if (fd < 0) {
        perror("Failed to open value file (read)");
        return -1;
    }
    char val_str[2] = {0};
    if (read(fd, val_str, 1) < 0) {
        perror("Failed to read GPIO value");
        close(fd);
        return -1;
    }
    close(fd);
    return atoi(val_str); // 返回0（低）或1（高）
}

// 设置GPIO值（用于LED）
int gpio_set_value(int gpio, int val) {
    char path[50] = {0};
    snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", gpio);
    int fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open value file (write)");
        return -1;
    }
    char val_str[2] = {0};
    snprintf(val_str, sizeof(val_str), "%d", val);
    if (write(fd, val_str, 1) < 0) {
        perror("Failed to set GPIO value");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

// 释放GPIO（用户空间→内核）
int gpio_unexport(int gpio) {
    char path[50] = {0};
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        perror("Failed to open unexport file");
        return -1;
    }
    char gpio_str[10] = {0};
    snprintf(gpio_str, sizeof(gpio_str), "%d", gpio);
    if (write(fd, gpio_str, strlen(gpio_str)) < 0) {
        perror("Failed to unexport GPIO");
        close(fd);
        return -1;
    }
    close(fd);
    return 0;
}

// -------------- 按键检测（带消抖） --------------
// 返回值：0=未按，1=按下，2=长按
int key_detect(int key_gpio) {
    static int last_state = 1; // 初始状态：未按（高电平）
    int current_state = gpio_get_value(key_gpio);

    // 状态无变化，返回未按
    if (current_state == last_state) {
        return 0;
    }

    // 状态变化，消抖延时10ms
    usleep(DEBOUNCE_MS * 1000);
    current_state = gpio_get_value(key_gpio);
    last_state = current_state;

    // 消抖后确认按下（高→低）
    if (current_state == 0) {
        // 记录按下开始时间，判断是否为长按
        clock_t press_start = clock();
        // 持续检测按下状态，超过1秒则判定为长按
        while (gpio_get_value(key_gpio) == 0) {
            clock_t press_now = clock();
            double press_diff = (double)(press_now - press_start) / CLOCKS_PER_SEC * 1000;
            if (press_diff >= LONG_PRESS_MS) {
                return 2; // 长按
            }
            usleep(10000); // 10ms检测一次
        }
        return 1; // 短按（按下后抬起）
    }
    return 0; // 未按（低→高，即抬起）
}

// -------------- 退出清理函数（Ctrl+C触发） --------------
void signal_handler(int sig) {
    printf("\nExiting... Cleaning GPIO resources\n");
    // 熄灭所有LED
    gpio_set_value(LED1_GPIO, 0);
    gpio_set_value(LED2_GPIO, 0);
    // 释放GPIO
    gpio_unexport(KEY3_GPIO);
    gpio_unexport(LED1_GPIO);
    gpio_unexport(LED2_GPIO);
    exit(0);
}

// -------------- 主函数逻辑 --------------
int main() {
    // 注册退出信号（Ctrl+C）
    signal(SIGINT, signal_handler);

    // 1. 初始化硬件GPIO
    printf("Initializing GPIOs...\n");
    // 初始化KEY3（输入）
    if (gpio_export(KEY3_GPIO) != 0 || gpio_set_dir(KEY3_GPIO, "in") != 0) {
        printf("Failed to initialize KEY3. Maybe already exported? Try unexport first.\n");
        gpio_unexport(KEY3_GPIO);
        gpio_export(KEY3_GPIO);
        gpio_set_dir(KEY3_GPIO, "in");
    }
    // 初始化LED1（输出，初始灭）
    if (gpio_export(LED1_GPIO) != 0 || gpio_set_dir(LED1_GPIO, "out") != 0) {
        printf("Failed to initialize LED1.\n");
        return -1;
    }
    gpio_set_value(LED1_GPIO, 0);
    // 初始化LED2（输出，初始灭）
    if (gpio_export(LED2_GPIO) != 0 || gpio_set_dir(LED2_GPIO, "out") != 0) {
        printf("Failed to initialize LED2. Try replacing with LED4 (GPIO38)!\n");
        return -1;
    }
    gpio_set_value(LED2_GPIO, 0);

    // 2. 初始化变量
    int led1_blink_en = 0;    // LED1闪烁使能（0：关，1：开）
    int led2_blink_en = 0;    // LED2闪烁使能（0：关，1：开）
    int led2_interval = 0;    // LED2闪烁间隔（ms）
    int led1_state = 0;       // LED1当前状态（0：灭，1：亮）
    int led2_state = 0;       // LED2当前状态（0：灭，1：亮）
    clock_t led1_last_time = 0;// LED1上次切换时间
    clock_t led2_last_time = 0;// LED2上次切换时间
    clock_t lift_start = 0;   // LED2抬起后开始计时时间
    int lift_en = 0;          // LED2抬起计时使能（0：关，1：开）

    printf("Initialization done. Start detecting KEY3...\n");
    printf("Tips: Press KEY3 (short/long) to control LEDs. Press Ctrl+C to exit.\n");

    // 3. 主循环（检测按键+控制LED）
    while (1) {
        // ---------------- 按键检测 ----------------
        int key_state = key_detect(KEY3_GPIO);
        switch (key_state) {
            case 1: // 短按（按下后抬起）
                printf("KEY3 short pressed → LED2 start fast blinking (200ms)\n");
                led2_blink_en = 1;
                led2_interval = LED2_FAST_MS;
                led2_last_time = clock(); // 重置LED2闪烁计时
                lift_en = 0; // 按下时，取消抬起计时
                break;
            case 2: // 长按（超过1秒）
                printf("KEY3 long pressed → LED1 start blinking (1s)\n");
                led1_blink_en = 1;
                led1_last_time = clock(); // 重置LED1闪烁计时
                // 同时启动LED2快速闪烁
                led2_blink_en = 1;
                led2_interval = LED2_FAST_MS;
                led2_last_time = clock();
                lift_en = 0;
                break;
            case 0: // 未按或抬起（需判断是否从按下→抬起）
                // 若之前LED2在闪烁且未计时，说明刚抬起
                if (led2_blink_en == 1 && lift_en == 0 && gpio_get_value(KEY3_GPIO) == 1) {
                    printf("KEY3 lifted → LED2 switch to slow blinking (1s), start 5s timer\n");
                    led2_interval = LED2_SLOW_MS;
                    led2_last_time = clock();
                    lift_start = clock(); // 开始5秒计时
                    lift_en = 1; // 使能抬起计时
                }
                break;
        }

        // ---------------- LED1控制（长按触发后交替闪烁） ----------------
        if (led1_blink_en == 1) {
            clock_t now = clock();
            double time_diff = (double)(now - led1_last_time) / CLOCKS_PER_SEC * 1000;
            if (time_diff >= LED1_INTERVAL) {
                led1_state = !led1_state; // 切换状态
                gpio_set_value(LED1_GPIO, led1_state);
                led1_last_time = now; // 更新上次切换时间
            }
        }

        // ---------------- LED2控制（按下快速/抬起慢速，5秒后灭） ----------------
        if (led2_blink_en == 1) {
            // 先判断抬起后是否超过5秒
            if (lift_en == 1) {
                clock_t now = clock();
                double lift_diff = (double)(now - lift_start) / CLOCKS_PER_SEC * 1000;
                if (lift_diff >= LIFT_OFF_MS) {
                    printf("5s passed after lifting → LED2 turned off\n");
                    led2_blink_en = 0;
                    led2_state = 0;
                    gpio_set_value(LED2_GPIO, 0);
                    lift_en = 0;
                    continue; // 跳过后续闪烁逻辑
                }
            }
            // 闪烁逻辑（按间隔切换状态）
            clock_t now = clock();
            double time_diff = (double)(now - led2_last_time) / CLOCKS_PER_SEC * 1000;
            if (time_diff >= led2_interval) {
                led2_state = !led2_state;
                gpio_set_value(LED2_GPIO, led2_state);
                led2_last_time = now;
            }
        }

        usleep(10000); // 10ms循环一次，降低CPU占用
    }

    return 0;
}