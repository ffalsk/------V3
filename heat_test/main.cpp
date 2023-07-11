#include <iostream>
#include <fstream>
#include <Windows.h>
using namespace std;
#define shoot_speed 15
#define MAX_HISTROY 10
#define Fliter_windowSize 5
uint16_t data_process(uint16_t* data);

int main()
{
	ifstream readFile;
	uint16_t* data = new uint16_t[1e6];
	memset(data, 0, static_cast<size_t>(1e6) * 2);
	readFile.open("shoot.csv", ios::in);

	if (readFile.is_open())
	{
		cout << "文件打开成功！" << endl;
		string buff = { 0 };
		int i = 0;
		while (readFile >> buff)
		{
			data[i] = abs(atoi(buff.c_str()));
			i++;
		}
	}
	else
	{
		cout << "文件打开失败！" << endl;
	}
	readFile.close();
	uint16_t shoot_count;
	shoot_count = data_process(data);
	cout << shoot_count << endl;
	system("pause");
	return 0;
}
uint16_t data_process(uint16_t* data)
{
	static bool bullet_waiting_confirm = false;//等待比较器确认
	static uint16_t shoot_count = 0;
	uint16_t data_histroy[MAX_HISTROY];//做循环队列
	static uint8_t head = 0, rear = 0;
	float moving_average[2];
	uint8_t data_num;
	float derivative;
	for (int i = 0; i < 1e6; i++)
	{
		data[i] = abs(data[i]);
		/*入队*/
		data_histroy[head] = data[i];
		head++;
		head %= MAX_HISTROY;
		/*判断队列数据量*/
		data_num = (head - rear + MAX_HISTROY) % MAX_HISTROY;
		if (data_num >= Fliter_windowSize + 1)//队列数据量满足要求
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
					cout << i << endl;
					shoot_count++;
					bullet_waiting_confirm = false;
				}
			}
			rear++;
			rear %= MAX_HISTROY;
		}
	}
	return shoot_count;
}