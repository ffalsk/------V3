# 本地热量管理V3.1

------

采用摩擦轮转速变化作为发弹判断依据

### 数据处理：

  ![流程](本地热量管理V3.assets/流程-1688969569747-2.png)

### 效果：

###### 18m/s

![image-20230710141540271](本地热量管理V3.assets/image-20230710141540271-1688969747425-4.png)

![image-20230710141555888](本地热量管理V3.assets/image-20230710141555888-1688969757049-6.png)

###### 30m/s

![image-20230710141623797](本地热量管理V3.assets/image-20230710141623797-1688969784763-8.png)

![image-20230710141631385](本地热量管理V3.assets/image-20230710141631385-1688969792274-10.png)

![image-20230710141638254](本地热量管理V3.assets/image-20230710141638254-1688969799083-12.png)

### 主函数(可加在shoot_task末尾)

```c
uint32_t shoot_count = 0;
void Shoot_Fric_data_process(void)
{
	/*----------------------------------变量常量------------------------------------------*/
	static bool bullet_waiting_confirm = false;										 // 等待比较器确认
	uint8_t shoot_speed = Referee_Inf.game_robot_state.shooter_id1_17mm_speed_limit; // 获取弹速
	int16_t data = CAN_Shoot[1].Current_Speed;										 // 获取摩擦轮转速
	static uint16_t data_histroy[MAX_HISTROY];										 // 做循环队列
	static uint8_t head = 0, rear = 0;												 // 队列下标
	float moving_average[2];														 // 移动平均滤波
	uint8_t data_num;																 // 循环队列元素个数
	float derivative;																 // 微分
	/*-----------------------------------逻辑控制-----------------------------------------*/
	data = abs(data);
	/*入队*/
	data_histroy[head] = data;
	head++;
	head %= MAX_HISTROY;
	/*判断队列数据量*/
	data_num = (head - rear + MAX_HISTROY) % MAX_HISTROY;
	if (data_num >= Fliter_windowSize + 1) // 队列数据量满足要求
	{
		moving_average[0] = 0;
		moving_average[1] = 0;
		/*同时计算两个滤波*/
		for (uint8_t i = rear, j = rear + 1, index = rear; index < rear + Fliter_windowSize; i++, j++, index++)
		{
			i %= MAX_HISTROY;
			j %= MAX_HISTROY;
			moving_average[0] += data_histroy[i];
			moving_average[1] += data_histroy[j];
		}
		moving_average[0] /= Fliter_windowSize;
		moving_average[1] /= Fliter_windowSize;
		/*滤波求导*/
		derivative = moving_average[1] - moving_average[0];
		/*导数比较*/
		if (derivative < -shoot_speed * 2)
		{
			bullet_waiting_confirm = true;
		}
		else if (derivative > -shoot_speed * 1.35)
		{
			if (bullet_waiting_confirm == true)
			{
				local_heat += One_bullet_heat; // 确认打出
				shoot_count++;
				bullet_waiting_confirm = false;
			}
		}
		rear++;
		rear %= MAX_HISTROY;
	}
}
```

**主函数执行放到main.c定时器中**

![image-20230710142207516](本地热量管理V3.assets/image-20230710142207516-1688970128563-14.png)

```c
/* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6)
  {
    /*-------------------------------------------热量控制部分---------------------------------------------*/
    local_heat -= (Referee_Inf.game_robot_state.shooter_id1_17mm_cooling_rate / 1000.0f); // 1000Hz冷却
    if (local_heat < 0)
    {
      local_heat = 0;
    }
    Shoot_Fric_data_process();
    /*-------------------------------------------IMU部分---------------------------------------------*/
    mpu_data.imu_tick_counts++;
    Yaw_Angle += mpu_data.processed_data.wz * 0.0576f;
  }
  /* USER CODE END Callback 1 */
```

### **热量控制部分**

![image-20230710142341567](本地热量管理V3.assets/image-20230710142341567-1688970222625-16.png)

```c
if (Referee_Inf.game_robot_state.shooter_id1_17mm_cooling_limit - local_heat <= heat_control) // 剩余热量小于留出的热量
	{
		Shoot_Speed = 0;
	}
```

### 其他一些变量的初始化

![image-20230710142720922](本地热量管理V3.assets/image-20230710142720922-1688970442301-18.png)

```c
// referee_info 数据类型
Referee_Info Referee_Inf = {0};
int heat_control = 20;    // 热量控制
float heat_remain = 0;    // 剩余热量
float local_heat = 0;     // 本地热量
int One_bullet_heat = 10; // 打一发消耗热量
```

**heat_control的值经过测试，20是个不错的选项**

**heat_control的值经过测试，20是个不错的选项**

**heat_control的值经过测试，20是个不错的选项**

##### **在shoot_task.h中**

![image-20230711142210530](本地热量管理V3.assets/image-20230711142210530-1689056531757-1.png)

```c
#include "main.h"
/*计算弹速相关*/
#define MAX_HISTROY 10
#define Fliter_windowSize 5.0f
void Shoot_Fric_data_process(void);
#endif
```



**每个车摩擦轮体质不一样，如果有必要，需要采集摩擦轮转速进行分析**
