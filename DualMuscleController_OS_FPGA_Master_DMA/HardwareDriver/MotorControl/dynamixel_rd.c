#include "dynamixel_rd.h"
//#include "dynamixel_protocol.h"
#include "stdio.h"
#include "stdint.h"
//#include "CMD_PC.h"
//#include "avalible.h"

#define Uart_Blk_Length 80
//Header -- Reserved -- Packet Length -- Instruction -- Start Address -- Data Length -- 16bit CRC
#define Error_Length 7
const unsigned char Dynamixel_Header[2] = {
  0xaa, 0xbb
};
const unsigned char Dynamixel_Error_Incomplete[Error_Length] = {0xAA, 0xBB, 0x00, 0x03, 0x66, 0x01, 0x75};

const unsigned char Dynamixel_Error_overrange[Error_Length] = {0xAA, 0xBB, 0x00, 0x03, 0x66, 0x02, 0x76};

extern unsigned short Dynamixel_crc_table[256];

//const unsigned short Dynamixel_crc_table[256] = {
//  0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
//  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
//  0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
//  0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
//  0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
//  0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
//  0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
//  0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
//  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
//  0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
//  0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
//  0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
//  0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
//  0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
//  0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
//  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
//  0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
//  0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
//  0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
//  0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
//  0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
//  0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
//  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
//  0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
//  0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
//  0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
//  0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
//  0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
//  0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
//  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
//  0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
//  0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
//};

unsigned char Data_Buffer[90]={0};

unsigned char Blk_Num = 0;
unsigned char Data_Blk[Uart_Blk_Length]={0};

//��������
//0  1     2       3         4           5              ....
//aa bb  reserved length Instruction  Payload
unsigned int Dynamixel_Master_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data)
{
  //��ַ�����ݳ��ȣ�����
  unsigned char  i;
  unsigned char Data2[1];
  unsigned int Return_State = Dynamixel_State_Error;
  for (i = 0; i < Data_Length; i++)
  {
    if (Blk_Num < 2)
    {
      if ((unsigned char)Data[i] == Dynamixel_Header[Blk_Num])
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }
      else Blk_Num = 0;
    }

    else if (Blk_Num == 2)
    {
      if ((unsigned char)Data[i] == Dynamixel_Reserved)
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }
      else Blk_Num = 0;
    }

    else if (Blk_Num == 3) //Packet Length
    {

      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;
    }

    else if (Blk_Num == 4)//Instruction
    {
      if (((unsigned char)Data[i] == Dynamixel_Instruction_Return) || ((unsigned char)Data[i] == Dynamixel_Instruction_Error) ||
          ((unsigned char)Data[i] == Dynamixel_Instruction_Read) || ((unsigned char)Data[i] == Dynamixel_Instruction_Write))
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }

      else Blk_Num = 0;
    }

    else if (Blk_Num == 5) //Payload
    {
      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;

      if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Error)
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];

        Dynamixel_Value_Dackward(CheckSum(Data_Blk, Data_Blk[3] + 3), 1, Data2);

        if (Data2[0] == Data[Data_Blk[3] + 3])
        {
          Return_State = Dynamixel_State_Success;
          Return_Data[0] = (unsigned char)Data_Blk[5];//Payload

          Blk_Num = 0;
        }
      }

    }

    else if (Blk_Num < Data_Blk[3] + 3)
    {
      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;
      if (Blk_Num == Data_Blk[3] + 3)
      {
        Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Data_Blk, Data_Blk[3] + 3), 1, Data2);
        // Dynamixel_Value_Dackward(CheckSum(Data_Blk, Data_Blk[3] + 3), 1, Data2);

        if (Data2[0] == Data[Data_Blk[3] + 3])
        {
          Return_State = Dynamixel_State_Success;
          if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Error)//������Ϣ
          {
            Return_Data[0] = (unsigned char)Data_Blk[5];//Payload
          }
          else
          {
            Return_Data[0] = (unsigned char)Data_Blk[5];//Address
            Return_Data[1] = Data_Blk[3] - 3;//����
            for (char k = 0; k < Return_Data[1]; k++)
            {
              Return_Data[2 + k] = (unsigned char)Data_Blk[6 + k];
            }

          }
        }
        Blk_Num = 0;
      }
    }
    else if (Blk_Num >= Uart_Blk_Length)
      Blk_Num = 0;
  }

  return Return_State;
}


/*
*功能：通讯协议从机处理函数
 *参数：
 * 	Data：接收的原始数据
 *  Data_Length：原始数据长度
 *  Return_Data：提取的有效数据
 *	Return_Data[0]：读写操作（读或写）
 *	Return_Data[1]：操作的地址
 *	Return_Data[2]：操作的地址长度
 *	Return_Data[3...N]：写入的数据
 *返回：
 *	处理结果：Dynamixel_Return_Success接收的数据有效  Dynamixel_Return_Error数据无效
*/ 
unsigned int Dynamixel_Slave_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data)
{
  //地址，数据长度，数据
  unsigned char  i;
  unsigned char Data2[1];
  unsigned int Return_State = Dynamixel_State_Error;
  for (i = 0; i < Data_Length; i++)
  {
    if (Blk_Num < 2)
    {
      if ((unsigned char)Data[i] == Dynamixel_Header[Blk_Num])
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }
      else Blk_Num = 0;
    }

    else if (Blk_Num == 2)
    {
      if ((unsigned char)Data[i] == Dynamixel_Reserved)
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }
      else Blk_Num = 0;
    }

    else if (Blk_Num == 3) //Packet Length
    {

      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;
    }

    else if (Blk_Num == 4)//Instruction
    {
      if (((unsigned char)Data[i] == Dynamixel_Instruction_Read) || ((unsigned char)Data[i] == Dynamixel_Instruction_Write))
      {
        Data_Blk[Blk_Num] = (unsigned char)Data[i];
        Blk_Num++;
      }

      else Blk_Num = 0;
    }

    else if (Blk_Num == 5)//Address
    {

      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;

    }
    else if (Blk_Num >= Uart_Blk_Length)
      Blk_Num = 0;
    else if (Blk_Num < Data_Blk[3] + 3+1)
    {
      Data_Blk[Blk_Num] = (unsigned char)Data[i];
      Blk_Num++;
      if (Blk_Num == Data_Blk[3] + 3 +1)
      {
        Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Data_Blk, Data_Blk[3] + 3), 1, Data2);
        //   Dynamixel_Value_Dackward(CheckSum(Data_Blk, Data_Blk[3] + 3), 1, Data2);

      if ((Data2[0] == Data[i])||(Data[Data_Blk[3] + 3]==0xff))
        {
          Return_State = Dynamixel_State_Success;
          if ((unsigned char)Data_Blk[4] == Dynamixel_Instruction_Read)
          {
            //printf("Read data:instruction_Address \r\n");
            Return_Data[0] = (unsigned char)Data_Blk[4];//instruction
            Return_Data[1] = (unsigned char)Data_Blk[5];//Address
            Return_Data[2] = (unsigned char)Data_Blk[6];//待读取长度

            for (char k = 0; k < 3; k++)
            {
             // printf("0x%X ", Return_Data[k]);
            }
          }
          else//写指令
          {
            //            printf("write data:instruction_Address \r\n");
            Return_Data[0] = (unsigned char)Data_Blk[4];//instruction
            Return_Data[1] = (unsigned char)Data_Blk[5];//Address
            Return_Data[2] = Data_Blk[3] - 3;//写入长度

						
            for (char k = 0; k < Return_Data[2]; k++)
            {
              Return_Data[3 + k] = (unsigned char)Data_Blk[6 + k];
              //              printf("0x%X ", Return_Data[3 + k]);
            }
          }
        }
				Data_Blk[Blk_Num] =0;
				Data_Blk[Blk_Num-1] =0;
				Data_Blk[Blk_Num] =0;
        Blk_Num = 0;
				Data[0] = 0;
				break;
				
      }
    }

  }

  return Return_State;
}
//
unsigned char Dynamixel_Slave_Send( unsigned char *DataBuffer, unsigned char Address, unsigned char data_length, unsigned char *Return_Data)
{
  unsigned char i = 0;
  unsigned char Length = 0;
  unsigned char Data[1];
  //Header
  for (i = 0; i < 2; i++)
  {
    Return_Data[Length] = Dynamixel_Header[i];
    Length++;
  }
  //Reserved
  Return_Data[Length] = Dynamixel_Reserved;
  Length++;

  //Packet Length
  Return_Data[Length] = data_length + 3;
  Length++;
  //Instruction
  Return_Data[Length] = Dynamixel_Instruction_Return;
  Length++;

  //Data
  Return_Data[Length] = Address;
  Length++;
  for (i = Address; i < data_length + Address; i++)
  {
    Return_Data[Length] = DataBuffer[i];
    Length++;
  }

  // Dynamixel_Value_Dackward(CheckSum(Return_Data, Length), 1, Data);
  Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Return_Data, Length), 1, Data);
  Return_Data[Length] = Data[0];
  Length++;
	
	
	Return_Data[Length] = 0x00;
  Length++;
	Return_Data[Length] = 0x00;
  Length++;

  return Length;

}
//��������
unsigned char Dynamixel_Master_Send(unsigned char Instruction, unsigned short Address, unsigned int *Value,
                                    unsigned char Value_Length, unsigned char *Return_Data)
{
  unsigned char i = 0;
  unsigned char Data4[4];
	unsigned char Data2[2];
  unsigned char Data[1];
  unsigned char Length = 0;

  //	Send_Data = (unsigned char*)malloc(sizeof(unsigned char) *(Value_Length+5+7));

  //Header
  for (i = 0; i < 2; i++)
  {
    Return_Data[Length] = Dynamixel_Header[i];
    Length++;
  }

  //Reserved
  Return_Data[Length] = Dynamixel_Reserved;
  Length++;

  //Packet Length
  if (Instruction == 0x02)
  {
    Return_Data[Length] = 0x04;
    Length++;
  }
  else//0x03
  {

      Return_Data[Length] = Value_Length + 3;
      Length++;

  }

  //Instruction
  Return_Data[Length] = Instruction;
  Length++;

  //Start Address
  Return_Data[Length] = Address;
  Length++;

  //Data
  if (Instruction == 0x02)
  {
    Return_Data[Length] = Value_Length;
    Length++;
  }
  else//0x03
  {
    if (Address == Dynamixel_Address_PresentCurrent2)
    {
	  if(Value_Length == 12)//4+8
      {
        for (int j = 0; j < 2; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 2, Data2);
          for (i = 0; i < 2; i++)
          {
            Return_Data[Length] = Data2[i];
            Length++;
          }
        }
        for (int j = 2; j < 4; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 4, Data4);
          for (i = 0; i < 4; i++)
          {
            Return_Data[Length] = Data4[i];
            Length++;
          }
        }
      }
      else if (Value_Length == 16)//4+8+4
      {
        for (int j = 0; j < 2; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 2, Data2);
          for (i = 0; i < 2; i++)
          {
            Return_Data[Length] = Data2[i];
            Length++;
          }
        }
        for (int j = 2; j < 4; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 4, Data4);
          for (i = 0; i < 4; i++)
          {
            Return_Data[Length] = Data4[i];
            Length++;
          }
        }
        for (int j = 4; j < 6; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 2, Data2);
          for (i = 0; i < 2; i++)
          {
            Return_Data[Length] = Data2[i];
            Length++;
          }
        }
      }
    }
	    else if(Address == Dynamixel_Address_GoalCurrent1)
    {
        for (int j = 0; j < 4; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 2, Data2);
          for (i = 0; i < 2; i++)
          {
            Return_Data[Length] = Data2[i];
            Length++;
          }
        }
				Return_Data[Length] = Value[4];
        Length++;
    }
    else if(Address == Dynamixel_Address_GoalPosition1)
    {
        for (int j = 0; j < 4; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 4, Data4);
          for (i = 0; i < 4; i++)
          {
            Return_Data[Length] = Data4[i];
            Length++;
          }
        }
								Return_Data[Length] = Value[4];
        Length++;
    }
    else if(Address == Dynamixel_Address_GoalVelocity1)
    {
        for (int j = 0; j < 4; j++)
        {
          Dynamixel_Value_Dackward(Value[j], 2, Data2);
          for (i = 0; i < 2; i++)
          {
            Return_Data[Length] = Data2[i];
            Length++;
          }
        }
								Return_Data[Length] = Value[4];
        Length++;
    }
    else//
    {
      Dynamixel_Value_Dackward(Value[0], Value_Length, Data4);
      for (i = 0; i < Value_Length; i++)
      {
        Return_Data[Length] = Data4[i];
        Length++;
      }
    }
  }

  //CRC
  Dynamixel_Value_Dackward(Dynamixel_update_crc(0, Return_Data, Length), 1, Data);
  // Dynamixel_Value_Dackward(CheckSum(Return_Data, Length), 1, Data);
  Return_Data[Length] = Data[0];
  Length++;

  // printf("send data is:\r\n");
  for (int i = 0; i < Length; i++)
  {
    //  printf("%02x \n", Return_Data[i]);
  }

  return Length;

}

int Dynamixel_Value_Forward(unsigned char *data_blk_ptr, unsigned char data_blk_size_start, unsigned char data_blk_size_end)
{
  unsigned int Value = 0x000000;
  unsigned char i;
  //if(data_blk_size_start == data_blk_size_end)
  //return Dynamixel_Return_Success;
  for (i = data_blk_size_end; i >= data_blk_size_start; i--)
  {
    Value = Value << 8;
    Value = Value | data_blk_ptr[i];
  }
	if(data_blk_size_end-data_blk_size_start<2)
	{
		if(Value>32767)
		{
			Value -= (0xffff + 1);
		}
	}
  else if (Value > 0xffffffff/2 )
    Value -= (0xffffffff + 1);

  return Value;
}

void  Dynamixel_Value_Dackward(int Value , unsigned char data_blk_size , unsigned char *Return_data)
{
  unsigned char i;
  for (i = 0; i < data_blk_size; i++)
  {
    Return_data[i] = Value & 0xff;
    Value = Value >> 8;
  }
}

unsigned short Dynamixel_update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size)
{
  unsigned short i, j;
  for (j = 0; j < data_blk_size; j++)
  {
    i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ Dynamixel_crc_table[i];
  }

  return crc_accum;
}

unsigned char  CheckSum(unsigned char uData[], unsigned char leng)
{
  char checksum = uData[0];
  for (int i = 0; i < leng; i++)
  {
    checksum ^= uData[i];
  }

  return checksum;
}

/*
 *功能：把数据写入对应的地址中，供PC读取
 *参数：
 * 	Address：地址
 *  Value：数值
 *  Length：数据长度
 *返回：无
*/ 
void Dynamixel_Buffer_Write(unsigned int Address, int Value, unsigned char Length)
{
	unsigned char data4[4];
	unsigned int i;
	Dynamixel_Value_Dackward(Value,Length,data4);
	for(i=0;i<Length;i++)
	{
		Data_Buffer[Address+i] = data4[i];		
	}
}
