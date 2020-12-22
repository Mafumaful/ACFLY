# Drv_SDI.cpp

```c
static void SDI_Server(void* pvParameters)
{
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头'A' 'C'
																	//1：接收1字节消息类别
																	//2：接收1字节消息长度
																	//3：接收数据包内容
																	//4：接收2字节校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 4*6
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sumA;
		static uint8_t sumB;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*状态机变量*/
	
	while(1)
	{
		uint8_t r_data;
		if( Read_Uart3( &r_data, 1, 2, 0.5 ) )
		{
			switch( rc_step1 )
			{
				case 0 :
					//接收包头'A''C'
					if( rc_step2 == 0 )
					{
						if( r_data == 'A' )
							rc_step2 = 1;
					}
					else
					{
						if( r_data == 'C' )
						{
							rc_step1 = 1;
							rc_step2 = 0;
							sumA = sumB = 0;
						}
						else
							rc_step2 = 0;
					}
					break;
					
				case 1:
					//接收消息类别
					msg_type = r_data;
					sumA += r_data;
					sumB += sumA;
					rc_step1 = 2;
					rc_step2 = 0;
					break;
				
				case 2:
					//接收消息长度
					if( r_data > MAX_SDI_PACKET_SIZE )
					{
						reset_SDI_RC;
						break;
					}
					msg_length = r_data;
					sumA += r_data;
					sumB += sumA;
					if( msg_length == 0 )
						rc_step1 = 4;
					else
						rc_step1 = 3;
					rc_step2 = 0;
					break;
					
				case 3:
					//接收数据包
					msg_pack[ rc_step2 ] = r_data;				
					sumA += r_data;
					sumB += sumA;
					++rc_step2;
					if( rc_step2 >= msg_length )
					{
						rc_step1 = 4;
						rc_step2 = 0;
					}
					break;
					
				case 4:
					//接收校验位
					if( rc_step2 == 0 )
					{
						if( sumA != r_data )
						{
							reset_SDI_RC;
							break;
						}
						rc_step2 = 1;
					}
					else
					{
						if( sumB == r_data )
						{
							ModeMsg msg;
							msg.cmd = msg_type;
							for( uint16_t i = 0; i < 8*4; ++i )
							{
								if( i < msg_length )
									((uint8_t*)(&msg.params[0]))[i] = msg_pack[i];
								else
									((uint8_t*)(&msg.params[0]))[i] = 0;
							}
							SendMsgToMode( msg, 0 );
						}
						reset_SDI_RC;
					}
					break;					
			}					
		}
	}
}
```

## rc_step

这个函数的目的是接受数据并将数据储存在相应的目标位置，接下来我将代码详细介绍：

```c
static uint8_t rc_step1 = 0;
```

解释接收函数的步骤用来是用来分布操作的关键，函数将帧分为了帧头，帧类型，数据长度，数据，校验这五个部分，所以对应了5个case。

## Read_Uart3

``` c
Read_Uart3( &r_data, 1, 2, 0.5 )
```

Uart3配置，主要设置一些时间常数啥的

## case0

``` c
				case 0 :
					//接收包头'A''C'
					if( rc_step2 == 0 )
					{
						if( r_data == 'A' )
							rc_step2 = 1;
					}
					else
					{
						if( r_data == 'C' )
						{
							rc_step1 = 1;
							rc_step2 = 0;
							sumA = sumB = 0;
						}
						else
							rc_step2 = 0;
					}
					break;
```

主要任务是接收帧头，帧头的格式为AC，如果接收到了帧头，则将step设置成下一步，并将接收数据帧步骤设置成0，将校验位赋值0（开始奇/偶校验），如果在接收到“A”的帧头后，接下来传输的数据不是“C”（说明不是帧头或者是传输出错），则将数据帧步骤设置成0。

## case1

```c
				case 1:
					//接收消息类别
					msg_type = r_data;
					sumA += r_data;
					sumB += sumA;
					rc_step1 = 2;
					rc_step2 = 0;
					break;
```

这里用来判断消息类别，第三个数据是用来传输消息的类别的。接收到了消息类别以后，计算校验位并跳转到下一步。这里又一次进行了一个数据帧清零的操作，说明只要在数据帧前面的步骤里面，为了保险起见，我们都需要将数据帧清零。

## case2

``` c
				case 2:
					//接收消息长度
					if( r_data > MAX_SDI_PACKET_SIZE )
					{
						reset_SDI_RC;
						break;
					}
					msg_length = r_data;
					sumA += r_data;
					sumB += sumA;
					if( msg_length == 0 )
						rc_step1 = 4;
					else
						rc_step1 = 3;
					rc_step2 = 0;
					break;
```

显示判断一下数据长度在不在合理范围以内，若数据在合理的范围以内的话，开始读取数据长度，计算奇偶校验，如果数据帧为0的话，说明没有数据帧，直接跳过数据帧检测。说明接收的数据可能会等于0。在case末尾，将数据帧步骤设置为0（as usual）。

## case3

```c
				case 3:
					//接收数据包
					msg_pack[ rc_step2 ] = r_data;				
					sumA += r_data;
					sumB += sumA;
					++rc_step2;
					if( rc_step2 >= msg_length )
					{
						rc_step1 = 4;
						rc_step2 = 0;
					}
					break;
```

这是最关键的一步目的是为了处理数据，这个数据是由msg_pack来接收当前所接收到的数据。在这个程序里面，每一次进入都会将数据帧步骤+1。在接收到符合长度的数据以后，我们需要将步骤设置为下一步（进行校验）。然后将数据帧步骤清零。

## case4

```c
				case 4:
					//接收校验位
					if( rc_step2 == 0 )
					{
						if( sumA != r_data )
						{
							reset_SDI_RC;
							break;
						}
						rc_step2 = 1;
					}
					else
					{
						if( sumB == r_data )
						{
							ModeMsg msg;
							msg.cmd = msg_type;
							for( uint16_t i = 0; i < 8*4; ++i )
							{
								if( i < msg_length )
									((uint8_t*)(&msg.params[0]))[i] = msg_pack[i];
								else
									((uint8_t*)(&msg.params[0]))[i] = 0;
							}
							SendMsgToMode( msg, 0 );
						}
						reset_SDI_RC;
					}
					break;	
```

这一步做了两件事情：

1. 和校验
2. 将数据帧传输给结构体，为了以后的调用

第一次校验如果失败，则重新将数据两个步骤都归为1（#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )），如果进行了第一次校验，说明数据帧步骤已经被设置成2了，就可以跳到第二次校验。在第二次校验里面，我们将结构体定义完全，然后传入一个消息类型msg.cmd，接下来就是将数据传入目标位置里面。一共需要循环8*4次，（一个float有四个uint8组成，这个数组一共有八个）,如果传输的数据不够的话，就直接存放0。

```c
struct ModeMsg
{
	uint8_t cmd_type;
	uint32_t cmd;
	float params[8];
};
```



最后，将两个步骤置0即可。