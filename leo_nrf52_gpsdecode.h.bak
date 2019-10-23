#ifndef LEO_NRF52_GPSDECODE_H
#define LEO_NRF52_GPSDECODE_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

#define g 9.8                       //后面可以改进通过 纬度求解当地 重力加速度g的数值
#define pi 3.1415926
#define Du2Hu pi/180


struct leo_mpu9255_config
{
    uint8_t     MagASAXYZ[3];       //XYZ轴磁强计 修正参数
    float       AccCoefficient;     //加计测量值的转换系数 根据量程不同而不同  量程：8G      对应 4096(LSB/g)
    float       GyrCoefficient;     //陀螺测量值的转换系数 根据量程不同而不同  量程：1000度  对应 32.8(LSB/(度/s))
    float       MagCoefficient;     //磁强计测量值的转换系数     0.6(uT/LSB)
    float       TemperatureCoefficient;     //温度测量值转换系数
};

struct leo_ADIS_config
{
    float       AccCoefficient;     //加计测量值的转换系数 根据量程不同而不同  量程：8G      对应 1.25(mg/LSB)
    float       GyrCoefficient;     //陀螺测量值的转换系数 根据量程不同而不同  量程：500度   对应 0.025((度/s)/LSB)
    float       TempCoefficient;    //温度测量值得转换系数  0.1 摄氏度/LSB
};
struct leo_mpu9255
{
    uint32_t    GpsWeekSeconds;
    uint16_t    MicroSeconds;       //每秒内的毫秒计数
    float       Acc[3];             // m/s
    float       Gyr[3];             // 弧度/s
    float       Mag[3];             // uT
    float       Temperature;        //温度 是摄氏度
};
struct leo_IMU_ADIS
{
    uint32_t    GpsWeekSeconds;
    uint16_t    MicroSeconds;       //每秒内的毫秒计数
    float       Acc[3];             // m/s
    float       Gyr[3];             // 弧度/s
    float       Temp;               // 温度，摄氏度
    uint16_t    nCounter;           //采样技术值
};

struct leo_gps
{
    uint32_t    GpsWeekSeconds;     //GPS 周内秒
    uint16_t    MicroSeconds;       //每秒内的毫秒计数
    float       Longitude;          //经度 度
    float       Latitude;           //纬度 度
    float       High;               //高度 米
    float       HDop;               //水平定位精度 米
};

struct leo_footpressure
{
    uint32_t    GpsWeekSeconds;     //GPS 周内秒
    uint16_t    MicroSeconds;       //每秒内的毫秒计数
    uint16_t    Point0;             //6点的压力值   未经过换算的 电压值
    uint16_t    Point1;             //7点的压力值
    uint16_t    Point2;             //5点的压力值
    uint16_t    Point3;             //2点的压力值
};

struct leo_uwb
{
    uint32_t    GpsWeekSeconds;     //GPS 周内秒
    uint16_t    MicroSeconds;       //每秒内的毫秒计数
    uint8_t     Counter;            //计数器
    float       Distance;           //距离
};



struct leo_wrongrecord
{
    uint32_t    GpsWeekSeconds;     //GPS 周内秒
    uint16_t    MicroSeconds;       //每秒内的毫秒计数
    uint8_t     MPU9255Acc;         //采集过程中，加计是否出错，1 代表出错；0 无错
    uint8_t     MPU9255Gry;         //采集过程中，陀螺是否出错，1 代表出错；0 无错
    uint8_t     MPU9255Mag;         //采集过程中，磁强计是否出错，1 代表出错；0 无错
};



struct minmea_float {
    int32_t     value;
    int32_t     scale;
};

/**
 * Convert a fixed-point value to a floating-point value.
 * Returns NaN for "unknown" values.
 */
static inline float minmea_tofloat(struct minmea_float *f)
{
    if (f->scale == 0)
        return (float)f->value;
    return (float) f->value / (float) f->scale;
}

/**
 * Convert a raw coordinate to a floating point DD.DDD... value.
 * Returns NaN for "unknown" values.
 */
static inline float minmea_tocoord(struct minmea_float *f)
{
    if (f->scale == 0)
        return (float)f->value;
    int32_t degrees = f->value / (f->scale * 100);
    int32_t minutes = f->value % (f->scale * 100);
    return (float) degrees + (float) minutes / (60 * f->scale);
}




void UTC2GPS(int year, int month, int day, int hour, int minute, int second, /*int *weekNo,*/ uint32_t *secondOfweek);

void leo_Decode_gps(struct leo_gps* mGPS,uint8_t* mBytes);

void leo_Decode_mpu9255(struct leo_mpu9255* mIMU,struct leo_mpu9255_config mCalibration,uint8_t* mBytes);

void leo_Decode_IMU_ADIS(struct leo_IMU_ADIS *mIMU,struct leo_ADIS_config mConfig, uint8_t *mBytes);

void leo_Decode_footpressure(struct leo_footpressure* mFootPressure,uint8_t* mBytes);

void leo_Decode_wrongrecord(struct leo_wrongrecord* mWrongRecord,uint8_t* mBytes);

void leo_Decode_uwb(struct leo_uwb *mUWBDistance, uint8_t *mBytes);
#ifdef __cplusplus
}
#endif


#endif // LEO_NRF52_GPSDECODE_H
