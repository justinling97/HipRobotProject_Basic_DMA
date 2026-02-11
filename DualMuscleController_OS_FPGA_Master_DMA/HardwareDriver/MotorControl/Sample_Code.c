//  /* Sample code provided by Jiawei to operate */
//	unsigned char str[100]={0};
//	unsigned char str_len = 0;
//	unsigned int aa[4] ={0};
//	/* Send Mode setting*/
//	aa[0] = 0; // 0 = position mode; 1= Velocity mode; 2 = current mode;
//	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Return,Dynamixel_Address_Mode,aa,1,str);
//	HAL_UART_Transmit_DMA(&huart2,str,str_len);
//	
//	/* Send Current setting*/
//	aa[0] = -100; // current of motor 1
//	aa[1] = 0;    // current of motor 2
//	aa[2] = 0;    // current of motor 3
//	aa[3] = 0;    // current of motor 4
//	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Return,Dynamixel_Address_GoalCurrent1,aa,8,str);
//	HAL_UART_Transmit_DMA(&huart2,str,str_len);
//	
//	/* Send Position setting*/
//	aa[0] = -100; // position of motor 1
//	aa[1] = 0;    // position of motor 2
//	aa[2] = 0;    // position of motor 3
//	aa[3] = 0;    // position of motor 4
//	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Return,Dynamixel_Address_GoalPosition1,aa,8,str);
//	HAL_UART_Transmit_DMA(&huart2,str,str_len);
//	
//	/* Send Velocity setting*/
//	/* P.S. Velocity need to be devided by 10 */
//	aa[0] = -100/10; // position of motor 1
//	aa[1] = 0/10;    // position of motor 2
//	aa[2] = 0/10;    // position of motor 3
//	aa[3] = 0/10;    // position of motor 4
//	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Return,Dynamixel_Address_GoalVelocity1,aa,8,str);
//	HAL_UART_Transmit_DMA(&huart2,str,str_len);