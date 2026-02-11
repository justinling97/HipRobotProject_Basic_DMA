#include "main.h"
#include "ControllerIMU.h"



extern IMUDataStruct IMU_Controller;


void Cmd_RxUnpack(uint8_t *buf, uint8_t Dlen)
{
    uint16_t ctl; // 数据订阅标识 标签0x11功能用到
    uint8_t L; // 标签0x11功能用到
    uint8_t tmpuint8_t;   // 1个8位数，方便后续使用与解析数据
    uint16_t tmpuint16_t; // 1个无符号16位数，方便后续使用与解析数据
    uint32_t tmpuint32_t; // 1个无符号32位数，方便后续使用与解析数据
    F32 tmpX, tmpY, tmpZ, tmpAbs; // 4个单精度浮点数，方便后续使用于解析数据

    switch (buf[0]) // bug[0]为数据体的第1字节表示功能标签
    {

    case 0x11: // 获取订阅的功能数据 回复或主动上报
        ctl = ((uint16_t)buf[2] << 8) | buf[1];// 字节[2-1] 为功能订阅标识，指示当前订阅了哪些功能


        L =7; // 从第7字节开始根据 订阅标识tag来解析剩下的数据
        if ((ctl & 0x0001) != 0)
        {// 加速度xyz 去掉了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); 
        }
        if ((ctl & 0x0002) != 0)
        {// 加速度xyz 包含了重力 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); 
					
						IMU_Controller.AX_Float=tmpX;
						IMU_Controller.AY_Float=tmpY;
						IMU_Controller.AZ_Float=tmpZ;
        }
        if ((ctl & 0x0004) != 0)
        {// 角速度xyz 使用时需*scaleAngleSpeed °/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; 
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; 
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngleSpeed; L += 2; 
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); 
					
						IMU_Controller.GX_Float=tmpX;
						IMU_Controller.GY_Float=tmpY;
						IMU_Controller.GZ_Float=tmpZ;
					
					
						
        }
        if ((ctl & 0x0008) != 0)
        {// 磁场xyz 使用时需*scaleMag uT
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; 
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; 
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleMag; L += 2; 
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ));

						IMU_Controller.HX_Float=tmpX;
						IMU_Controller.HY_Float=tmpY;
						IMU_Controller.HZ_Float=tmpZ;					
        }
        if ((ctl & 0x0010) != 0)
        {// 温度 气压 高度
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleTemperature; L += 2; 

            tmpuint32_t = (uint32_t)(((uint32_t)buf[L+2] << 16) | ((uint32_t)buf[L+1] << 8) | (uint32_t)buf[L]);
            tmpuint32_t = ((tmpuint32_t & 0x800000) == 0x800000)? (tmpuint32_t | 0xff000000) : tmpuint32_t;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpY = (S32)tmpuint32_t * scaleAirPressure; L += 3; 

            tmpuint32_t = (uint32_t)(((uint32_t)buf[L+2] << 16) | ((uint32_t)buf[L+1] << 8) | (uint32_t)buf[L]);
            tmpuint32_t = ((tmpuint32_t & 0x800000) == 0x800000)? (tmpuint32_t | 0xff000000) : tmpuint32_t;// 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
            tmpZ = (S32)tmpuint32_t * scaleHeight; L += 3; 
					
						IMU_Controller.Atmospheric_pressure_Float=(float)tmpY;
						IMU_Controller.Height_Float=(float)tmpZ;
        }
        if ((ctl & 0x0020) != 0)
        {// 四元素 wxyz 使用时需*scaleQuat
            tmpAbs = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; 
            tmpX =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; 
            tmpY =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; 
            tmpZ =   (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleQuat; L += 2; 
        }
        if ((ctl & 0x0040) != 0)
        {// 欧拉角xyz 使用时需*scaleAngle
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; 
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; 
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAngle; L += 2; 
					
						IMU_Controller.Roll_Float=tmpX;
						IMU_Controller.Pitch_Float=tmpY;
						IMU_Controller.Yaw_Float=tmpZ;
        }
        if ((ctl & 0x0080) != 0)
        {// xyz 空间位移 单位mm 转为 m
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; 
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; 
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) / 1000.0f; L += 2; 
        }
        if ((ctl & 0x0100) != 0)
        {// 活动检测数据
            tmpuint32_t = (uint32_t)(((uint32_t)buf[L+3]<<24) | ((uint32_t)buf[L+2]<<16) | ((uint32_t)buf[L+1]<<8) | ((uint32_t)buf[L]<<0)); L += 4; 
            tmpuint8_t = buf[L]; L += 1;

        }
        if ((ctl & 0x0200) != 0)
        {// 加速度xyz 去掉了重力且已转为导航系 使用时需*scaleAccel m/s
            tmpX = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpY = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpZ = (S16)(((S16)buf[L+1]<<8) | buf[L]) * scaleAccel; L += 2; 
            tmpAbs = sqrt(pow2(tmpX) + pow2(tmpY) + pow2(tmpZ)); 
        }
        if ((ctl & 0x0400) != 0)
        {// ADC的值
            tmpuint16_t = (uint16_t)(((uint16_t)buf[L+1]<<8) | ((uint16_t)buf[L]<<0)); L += 2; 
        }
        if ((ctl & 0x0800) != 0)
        {// GPIO1的值
            tmpuint8_t = buf[L]; L += 1;

        }
				
//				IMU_Controller.AX_Float
//				IMU_Controller.AY_Float
//				IMU_Controller.AZ_Float
//				
//				IMU_Controller.GX_Float
//				IMU_Controller.GY_Float
//				IMU_Controller.GZ_Float
//				
//				IMU_Controller.HX_Float
//				IMU_Controller.HY_Float
//				IMU_Controller.HZ_Float
//				
//				IMU_Controller.Roll_Float
//				IMU_Controller.Pitch_Float
//				IMU_Controller.Yaw_Float
				
        break;
   

    default:
        break;
    }
}