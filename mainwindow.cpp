#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QFile>
#include "leo_nrf52_gpsdecode.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_Quite_clicked()
{
//    QApplication* app;
//    app->exit(0);

    int year,month,day,hour,minute,second;
    year = 2018;
    month = 10;
    day = 27;
    hour = 11;
    minute = 18;
    second = 58;
    uint32_t WeekSecond;
    UTC2GPS(year,month,day,hour,minute,second,&WeekSecond);
    QString temp;
    temp.sprintf("%d",WeekSecond);
    QMessageBox::about(NULL, "提示",temp);


}

void MainWindow::on_pushButton_DataOpen_clicked()
{
    mDataPath_Open = QFileDialog::getOpenFileName(this,tr("选择需要解析的数据"),"E:\\2_WorkSpace_Leo\\0_Data",tr("Allfile(*.*);.dat"));
    ui->lineEdit_DataPathOpen->setText(mDataPath_Open);
//    QMessageBox::about(NULL, "提示",mDataPath_Open);
}

void MainWindow::on_pushButton_DataDecode_clicked()
{
    //准备====================获取数据路径
    QString mWrong("错误！");
    QString mFind(".");
    int m = mDataPath_Open.lastIndexOf(mFind);
    mDataPath_Save_MAG_Coe = mDataPath_Open.left(m);
    mDataPath_Save_MAG_Coe.append("_MAGCoe.txt");
    mDataPath_Save_IMU_A = mDataPath_Open.left(m);
    mDataPath_Save_IMU_A.append("_IMU_A.txt");
    mDataPath_Save_IMU_B = mDataPath_Open.left(m);
    mDataPath_Save_IMU_B.append("_IMU_B.txt");
    mDataPath_Save_GPS = mDataPath_Open.left(m);
    mDataPath_Save_GPS.append("_GPS.txt");
    mDataPath_Save_FootPressure = mDataPath_Open.left(m);
    mDataPath_Save_FootPressure.append("_FootPressure.txt");
    mDataPath_Save_UWB = mDataPath_Open.left(m);
    mDataPath_Save_UWB.append("_UWB.txt");

    //准备====================建立文件并打开
    QFile mFile_DataOpen(mDataPath_Open);
    if(!mFile_DataOpen.open(QIODevice::ReadOnly))
        QMessageBox::about(NULL, "提示",mWrong.append(mDataPath_Open));

    QFile mFile_DataSave_MAGCoe(mDataPath_Save_MAG_Coe);
    if(!mFile_DataSave_MAGCoe.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly))
        QMessageBox::about(NULL, "提示",mWrong.append(mDataPath_Save_MAG_Coe));

    QFile mFile_DataSave_IMU_A(mDataPath_Save_IMU_A);
    if(!mFile_DataSave_IMU_A.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly))
        QMessageBox::about(NULL, "提示",mWrong.append(mDataPath_Save_IMU_A));

    QFile mFile_DataSave_IMU_B(mDataPath_Save_IMU_B);
    if(!mFile_DataSave_IMU_B.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly))
        QMessageBox::about(NULL, "提示",mWrong.append(mDataPath_Save_IMU_B));

    QFile mFile_DataSave_GPS(mDataPath_Save_GPS);
    if(!mFile_DataSave_GPS.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly))
        QMessageBox::about(NULL, "提示",mWrong.append(mDataPath_Save_GPS));

    QFile mFile_DataSave_FootPressure(mDataPath_Save_FootPressure);
    if(!mFile_DataSave_FootPressure.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly))
        QMessageBox::about(NULL, "提示",mWrong.append(mDataPath_Save_FootPressure));

    QFile mFile_DataSave_UWB(mDataPath_Save_UWB);
    if(!mFile_DataSave_UWB.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::NewOnly))
        QMessageBox::about(NULL, "提示",mWrong.append(mDataPath_Save_UWB));
    //一切就绪====================开始
    uint32_t mGPSWeekSecond;
    uint8_t mHead_First,mHead_Second;
    char mChar,mEnd[1];
    struct leo_mpu9255_config mConfig;
    memset(&mConfig,0,sizeof(mConfig));
    //设置 数据转换参数
    mConfig.AccCoefficient = 4096.0;
    mConfig.GyrCoefficient = 32.8;
    mConfig.MagCoefficient = 0.6;
    while(!mFile_DataOpen.atEnd())
    {
        mFile_DataOpen.read(&mChar,1);
        switch ((uint8_t)mChar) {
        //磁强计 修正参数=======================================
        case 0xA1:
            mFile_DataOpen.read(&mChar,1);
            if((uint8_t)mChar == 0xA1)
            {
                char mConfigBytes[3];
                mFile_DataOpen.read(mConfigBytes,3);
                mFile_DataOpen.read(mEnd,1);
                if((uint8_t)mEnd[0]==0xFF)
                {
                    mConfig.MagASAXYZ[0] = (uint8_t)mConfigBytes[0];
                    mConfig.MagASAXYZ[1] = (uint8_t)mConfigBytes[1];
                    mConfig.MagASAXYZ[2] = (uint8_t)mConfigBytes[2];
                    QString temp;
                    temp.sprintf("磁强计校正参数：%x %x %x！",mConfig.MagASAXYZ[0],mConfig.MagASAXYZ[1],mConfig.MagASAXYZ[2]);
                    QMessageBox::about(NULL, "提示",temp);
                    QString temp_StringMAGCoe;
                    temp_StringMAGCoe.sprintf("%d %d %d\n",mConfig.MagASAXYZ[0],mConfig.MagASAXYZ[1],mConfig.MagASAXYZ[2]);
                    QByteArray temp_CharMAGCoe = temp_StringMAGCoe.toLatin1();
                    mFile_DataSave_MAGCoe.write(temp_CharMAGCoe.data(),temp_CharMAGCoe.length());
                }else
                    continue;
            }else
                continue;
            break;

        //GPS周内秒 数据===============================================
//        case 0xA0:
//            mFile_DataOpen.read(&mChar,1);
//            if((uint8_t)mChar == 0xA0)
//            {
//                mFile_DataOpen.read((char*)&mGPSWeekSecond,4);
//                mFile_DataOpen.read(mEnd,1);
//                if((uint8_t)mEnd[0]!=0xFF)
//                {
//                    QMessageBox::about(NULL, "提示","GPS周内秒数据错误！");
//                }
//            }else
//                continue;

//            break;

        //IMU_A 数据===============================================
        case 0xB1:
            mFile_DataOpen.read(&mChar,1);
            if((uint8_t)mChar == 0xB1)
            {
                char temp_IMUBytes[24] = {0};
                mFile_DataOpen.read(temp_IMUBytes,24);
                mFile_DataOpen.read(mEnd,1);
                if((uint8_t)mEnd[0]==0xFF)
                {
                    struct leo_mpu9255 temp_IMUData;
                    memset(&temp_IMUData,0,sizeof(temp_IMUData));
                    leo_Decode_mpu9255(&temp_IMUData,mConfig,(uint8_t*)temp_IMUBytes);
                    //此处进行数据存储
                    QString temp_StringIMU;
                    temp_StringIMU.sprintf("%d %d %f %f %f %f %f %f %f %f %f\n",temp_IMUData.GpsWeekSeconds,temp_IMUData.MicroSeconds,
                                           temp_IMUData.Acc[0],temp_IMUData.Acc[1],temp_IMUData.Acc[2],
                                           temp_IMUData.Gyr[0],temp_IMUData.Gyr[1],temp_IMUData.Gyr[2],
                                           temp_IMUData.Mag[0],temp_IMUData.Mag[1],temp_IMUData.Mag[2]);
                    QByteArray temp_CharIMU = temp_StringIMU.toLatin1();
                    mFile_DataSave_IMU_A.write(temp_CharIMU.data(),temp_CharIMU.length());
                }else
                    continue;

            }else
                continue;

            break;


         //压力传感器 数据===============================================
            // 数据*3.6/1024.0 是电压，还需要依据电阻 转换为压力值
         case 0xC1:
             mFile_DataOpen.read(&mChar,1);
             if((uint8_t)mChar == 0xC1)
             {
                 char temp_PREBytes[14] = {0};
                 mFile_DataOpen.read(temp_PREBytes,14);
                 mFile_DataOpen.read(mEnd,1);
                 if((uint8_t)mEnd[0]==0xFF)
                 {
                     struct leo_footpressure temp_PREData;
                     leo_Decode_footpressure(&temp_PREData,(uint8_t*)temp_PREBytes);
                     QString temp_StringPre;
                     temp_StringPre.sprintf("%d %d %d %d %d %d\n",temp_PREData.GpsWeekSeconds,temp_PREData.MicroSeconds,
                                             temp_PREData.Point0,temp_PREData.Point1,temp_PREData.Point2,temp_PREData.Point3);

                     QByteArray temp_CharPre = temp_StringPre.toLatin1();
                     mFile_DataSave_FootPressure.write(temp_CharPre.data(),temp_CharPre.length());
                 }else
                     continue;

             }else
                 continue;
             break;

        //GPS 数据===================================================
        case 0xD1:
            mFile_DataOpen.read(&mChar,1);
            if((uint8_t)mChar == 0xD1)
            {
                char temp_GPSBytes[38] = {0};
                mFile_DataOpen.read(temp_GPSBytes,38);
                mFile_DataOpen.read(mEnd,1);
                if((uint8_t)mEnd[0]==0xFF)
                {
                    struct leo_gps temp_GPSData;
                    memset(&temp_GPSData,0,sizeof(temp_GPSData));
                    leo_Decode_gps(&temp_GPSData,(uint8_t*)temp_GPSBytes);
                    QString temp_StringGPS;
                    temp_StringGPS.sprintf("%d %d %f %f %f %f\n",temp_GPSData.GpsWeekSeconds,temp_GPSData.MicroSeconds,
                                           temp_GPSData.Longitude,temp_GPSData.Latitude,temp_GPSData.High,temp_GPSData.HDop);
                    QByteArray temp_CharGPS = temp_StringGPS.toLatin1();
                    mFile_DataSave_GPS.write(temp_CharGPS.data(),temp_CharGPS.length());
                }else
                    continue;

            }else
                continue;
            break;

        //采集UWB测距 数据===============================================
        case 0xE1:
            mFile_DataOpen.read(&mChar,1);
            if((uint8_t)mChar == 0xE1)
            {
                char temp_UWBBytes[9] = {0};
                mFile_DataOpen.read(temp_UWBBytes,9);
                mFile_DataOpen.read(mEnd,1);
                if((uint8_t)mEnd[0]==0xFF)
                {
                    struct leo_uwb temp_UWBData;
                    memset(&temp_UWBData,0,sizeof(temp_UWBData));
                    leo_Decode_uwb(&temp_UWBData,(uint8_t*)temp_UWBBytes);
                    QString temp_StringUWB;
                    temp_StringUWB.sprintf("%d %d %d %f\n",temp_UWBData.GpsWeekSeconds,temp_UWBData.MicroSeconds,
                                           temp_UWBData.Counter,temp_UWBData.Distance);
                    QByteArray temp_CharUWB = temp_StringUWB.toLatin1();
                    mFile_DataSave_UWB.write(temp_CharUWB.data(),temp_CharUWB.length());
                }else
                    continue;

            }else
                continue;

            break;

        //采集错误 数据===============================================
        case 0xC3:

            break;

        default:
            break;
        }



    }









    //解析完成====================关闭文件并退出
    mFile_DataOpen.close();
    mFile_DataSave_MAGCoe.close();
    mFile_DataSave_IMU_A.close();
    mFile_DataSave_IMU_B.close();
    mFile_DataSave_GPS.close();
    mFile_DataSave_FootPressure.close();
    mFile_DataSave_UWB.close();
    QMessageBox::about(NULL, "提示","解析完成！！");


}
