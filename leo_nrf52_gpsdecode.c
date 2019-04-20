#include "leo_nrf52_gpsdecode.h"





void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek)
{
/*****协调世界时转换为GPS的周秒表示*****///输入时间应为协调世界时，即当地时间-8，返回时间为GPS周和周秒
    int DayofYear = 0;
    int DayofMonth = 0;


    for(int i = 1980; i < year; i++) //从1980年到当前年的上一年经过的天数
    {
        if ((i % 4 == 0 && i % 100 != 0) || i % 400 == 0)
        DayofYear += 366;
        else
        DayofYear += 365;
    }
    for(int i = 1; i < month; i++)//从一月到当前月的上一月经历的天数
    {
        if(i == 1 || i == 3 || i == 5 || i == 7 || i == 8 || i == 10 || i ==12)
            DayofMonth += 31;
        else
            if(i == 4 || i == 6 || i == 9 || i == 11)
                DayofMonth += 30;
            else
            {
                if((year % 4 == 0 && year % 100 != 0) || year % 400 == 0)
                    DayofMonth += 29;
                else
                    DayofMonth += 28;
            }
    }

    int Day;
    Day = DayofMonth + day + DayofYear-6;
    //*weekNo = Day/7;
    *secondOfweek = Day % 7 * 86400 + hour * 3600 + minute * 60 + second+18;//18表示跳秒
    return ;
}




void leo_Decode_gps(struct leo_gps* mGPS,uint8_t* mBytes)
{
    struct minmea_float mLong,mLat,mHigh,mHDop;

    memcpy(&(mGPS->GpsWeekSeconds),mBytes,4);
    memcpy(&(mGPS->MicroSeconds),mBytes+4,2);
    memcpy(&(mLong.value),mBytes+6,4);
    memcpy(&(mLong.scale),mBytes+10,4);
    memcpy(&(mLat.value),mBytes+14,4);
    memcpy(&(mLat.scale),mBytes+18,4);
    memcpy(&(mHigh.value),mBytes+22,4);
    memcpy(&(mHigh.scale),mBytes+26,4);
    memcpy(&(mHDop.value),mBytes+30,4);
    memcpy(&(mHDop.scale),mBytes+34,4);

//    mGPS->MicroSeconds = (*(mBytes+5)) | (*(mBytes+6)<<8);
//    mLong.value = (*(mBytes+7)) | (*(mBytes+8)<<8) | (*(mBytes+9)<<16) | (*(mBytes+10)<<24);
//    mLong.scale = (*(mBytes+11)) | (*(mBytes+12)<<8) | (*(mBytes+13)<<16) | (*(mBytes+14)<<24);
//    mLat.value = (*(mBytes+15)) | (*(mBytes+16)<<8) | (*(mBytes+17)<<16) | (*(mBytes+18)<<24);
//    mLat.scale = (*(mBytes+19)) | (*(mBytes+20)<<8) | (*(mBytes+21)<<16) | (*(mBytes+22)<<24);
//    mHigh.value = (*(mBytes+23)) | (*(mBytes+24)<<8) | (*(mBytes+25)<<16) | (*(mBytes+26)<<24);
//    mHigh.scale = (*(mBytes+27)) | (*(mBytes+28)<<8) | (*(mBytes+29)<<16) | (*(mBytes+30)<<24);

    mGPS->Longitude = minmea_tocoord(&mLong);
    mGPS->Latitude = minmea_tocoord(&mLat);
    mGPS->High = minmea_tofloat(&mHigh);
    mGPS->HDop = minmea_tofloat(&mHDop);

    return;
}



void leo_Decode_mpu9255(struct leo_mpu9255 *mIMU,struct leo_mpu9255_config mCalibration, uint8_t *mBytes)
{
    memcpy(&(mIMU->GpsWeekSeconds),mBytes,4);
    memcpy(&(mIMU->MicroSeconds),mBytes+4,2);
//    mIMU->MicroSeconds = (*(mBytes)) | (*(mBytes+1)<<8);

    int16_t AccX,AccY,AccZ;
    AccX = (*(mBytes+6) << 8) | *(mBytes+7);
    if(AccX & 0x8000) AccX-=65536;
    AccY= (*(mBytes+8) << 8) | *(mBytes+9);
    if(AccY & 0x8000) AccY-=65536;
    AccZ = (*(mBytes+10) << 8) | *(mBytes+11);
    if(AccZ & 0x8000) AccZ-=65536;
    mIMU->Acc[0] = ((float)AccX) / mCalibration.AccCoefficient * g;
    mIMU->Acc[1] = ((float)AccY) / mCalibration.AccCoefficient * g;
    mIMU->Acc[2] = ((float)AccZ) / mCalibration.AccCoefficient * g;

    int16_t Temperature;
    Temperature = (*(mBytes+12) << 8) | *(mBytes+13);
    mIMU->Temperature = ((float)Temperature) / mCalibration.TemperatureCoefficient + 21.0;

    int16_t GyrX,GyrY,GyrZ;
    GyrX = (*(mBytes+14) << 8) | *(mBytes+15);
    if(GyrX & 0x8000) GyrX-=65536;
    GyrY= (*(mBytes+16) << 8) | *(mBytes+17);
    if(GyrY & 0x8000) GyrY-=65536;
    GyrZ = (*(mBytes+18) << 8) | *(mBytes+19);
    if(GyrZ & 0x8000) GyrZ-=65536;
    mIMU->Gyr[0] = ((float)GyrX) / ((float)mCalibration.GyrCoefficient) ;
    mIMU->Gyr[1] = ((float)GyrY) / ((float)mCalibration.GyrCoefficient) ;
    mIMU->Gyr[2] = ((float)GyrZ) / ((float)mCalibration.GyrCoefficient) ;

    int16_t MagX,MagY,MagZ;
    MagX = (*(mBytes+22) << 8) | *(mBytes+21);
    if(MagX & 0x8000) MagX-=65536;
    MagY= (*(mBytes+24) << 8) | *(mBytes+23);
    if(MagY & 0x8000) MagY-=65536;
    MagZ = (*(mBytes+26) << 8) | *(mBytes+25);
    if(MagZ & 0x8000) MagZ-=65536;
//    int16_t MagX,MagY,MagZ;
//    MagX = (*(mBytes+18) << 8) | *(mBytes+19);
//    if(MagX & 0x8000) MagX-=65536;
//    MagY= (*(mBytes+20) << 8) | *(mBytes+21);
//    if(MagY & 0x8000) MagY-=65536;
//    MagZ = (*(mBytes+22) << 8) | *(mBytes+23);
//    if(MagZ & 0x8000) MagZ-=65536;
    MagX = MagX * (((float)mCalibration.MagASAXYZ[0]-128.0)*0.5/128.0+1);
    MagY = MagY * (((float)mCalibration.MagASAXYZ[1]-128.0)*0.5/128.0+1);
    MagZ = MagZ * (((float)mCalibration.MagASAXYZ[2]-128.0)*0.5/128.0+1);

    mIMU->Mag[0] = ((float)MagX) * ((float)mCalibration.MagCoefficient);
    mIMU->Mag[1] = ((float)MagY) * ((float)mCalibration.MagCoefficient);
    mIMU->Mag[2] = ((float)MagZ) * ((float)mCalibration.MagCoefficient);

    return;
}


void leo_Decode_IMU_ADIS(struct leo_IMU_ADIS *mIMU,struct leo_ADIS_config mConfig, uint8_t *mBytes)
{
    memcpy(&(mIMU->GpsWeekSeconds),mBytes,4);
    memcpy(&(mIMU->MicroSeconds),mBytes+4,2);

    int16_t GyrX,GyrY,GyrZ;
    GyrX = (*(mBytes+6) << 8) | *(mBytes+7);
    if(GyrX & 0x8000) GyrX-=65536;
    GyrY= (*(mBytes+8) << 8) | *(mBytes+9);
    if(GyrY & 0x8000) GyrY-=65536;
    GyrZ = (*(mBytes+10) << 8) | *(mBytes+11);
    if(GyrZ & 0x8000) GyrZ-=65536;
    mIMU->Gyr[0] = ((float)GyrX) * ((float)mConfig.GyrCoefficient) ;
    mIMU->Gyr[1] = ((float)GyrY) * ((float)mConfig.GyrCoefficient) ;
    mIMU->Gyr[2] = ((float)GyrZ) * ((float)mConfig.GyrCoefficient) ;

    int16_t AccX,AccY,AccZ;
    AccX = (*(mBytes+12) << 8) | *(mBytes+13);
    if(AccX & 0x8000) AccX-=65536;
    AccY = (*(mBytes+14) << 8) | *(mBytes+15);
    if(AccY & 0x8000) AccY-=65536;
    AccZ = (*(mBytes+16) << 8) | *(mBytes+17);
    if(AccZ & 0x8000) AccZ-=65536;

    mIMU->Acc[0] = ((float)AccX) * ((float)mConfig.AccCoefficient) * g/1000.0;
    mIMU->Acc[1] = ((float)AccY) * ((float)mConfig.AccCoefficient) * g/1000.0;
    mIMU->Acc[2] = ((float)AccZ) * ((float)mConfig.AccCoefficient) * g/1000.0;

    int16_t Temperature;
    Temperature = (*(mBytes+18) << 8) | *(mBytes+19);
    mIMU->Temp = ((float)Temperature) * mConfig.TempCoefficient;

    mIMU->nCounter = (*(mBytes+20) << 8) | *(mBytes+21);

    return;
}


void leo_Decode_footpressure(struct leo_footpressure *mFootPressure, uint8_t *mBytes)
{
    memcpy(&(mFootPressure->GpsWeekSeconds),mBytes,4);
    memcpy(&(mFootPressure->MicroSeconds),mBytes+4,2);
//    mFootPressure->MicroSeconds = (*(mBytes)) | (*(mBytes+1)<<8);
//    mFootPressure->Point0 = (*(mBytes+6)) | (*(mBytes+7)<<8);
//    mFootPressure->Point1 = (*(mBytes+8)) | (*(mBytes+9)<<8);
//    mFootPressure->Point2 = (*(mBytes+10)) | (*(mBytes+11)<<8);
//    mFootPressure->Point3 = (*(mBytes+12)) | (*(mBytes+13)<<8);
    memcpy(&(mFootPressure->Point0),mBytes+6,2);
    memcpy(&(mFootPressure->Point1),mBytes+8,2);
    memcpy(&(mFootPressure->Point2),mBytes+10,2);
    memcpy(&(mFootPressure->Point3),mBytes+12,2);
}


void leo_Decode_uwb(struct leo_uwb *mUWBDistance, uint8_t *mBytes)
{
    uint16_t temp;
    memcpy(&(mUWBDistance->GpsWeekSeconds),mBytes,4);
    memcpy(&(mUWBDistance->MicroSeconds),mBytes+4,2);
    memcpy(&(mUWBDistance->Counter),mBytes+6,1);
    memcpy(&temp,mBytes+7,2);
    mUWBDistance->Distance = temp/1000.0;
}


