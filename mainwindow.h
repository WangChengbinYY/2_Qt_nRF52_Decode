#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    QString mDataPath_Open;
    QString mDataPath_Save_MAGA_Coe;
    QString mDataPath_Save_MAGB_Coe;
    QString mDataPath_Save_IMU_A;
    QString mDataPath_Save_IMU_B;
    QString mDataPath_Save_FootPressure;
    QString mDataPath_Save_GPS;
    QString mDataPath_Save_UWB;




public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_Quite_clicked();

    void on_pushButton_DataOpen_clicked();

    void on_pushButton_DataDecode_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
