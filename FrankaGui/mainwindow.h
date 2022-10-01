#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "helpwindow.h"
#include "button_functions.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    HelpWindow *hWindow;

public slots:
    //Slider controls.
    void sliderPlus_1();
    void sliderMinus_1();
    void sliderPlus_2();
    void sliderMinus_2();
    void sliderPlus_3();
    void sliderMinus_3();
    void sliderPlus_4();
    void sliderMinus_4();
    void sliderPlus_5();
    void sliderMinus_5();
    void sliderPlus_6();
    void sliderMinus_6();
    void sliderPlus_7();
    void sliderMinus_7();

    void on_buttonHelp_clicked();
    void on_buttonAddPoint_clicked();
    void on_buttonDelete_clicked();
    void on_buttonChange_clicked();
    void on_buttonMoveDown_clicked();
    void on_buttonMoveUp_clicked();
    void on_buttonMoveJoints_clicked();
    void on_buttonOpenGripper_clicked();
    void on_buttonCloseGripper_clicked();
    void on_buttonHome_clicked();
    void on_buttonAddGripper_clicked();
    void on_buttonReset_clicked();
    void on_buttonStart_clicked();
    void on_buttonImpedance_clicked();
    void on_buttonAddCurrent_clicked();
    void on_buttonGetJoints_clicked();
    void on_buttonAddJointMotion_clicked();

    void on_slider_1_valueChanged(int value);
    void on_spinBox_1_valueChanged(double arg1);
    void on_slider_2_valueChanged(int value);
    void on_spinBox_2_valueChanged(double arg1);
    void on_slider_3_valueChanged(int value);
    void on_spinBox_3_valueChanged(double arg1);
    void on_slider_4_valueChanged(int value);
    void on_spinBox_4_valueChanged(double arg1);
    void on_slider_5_valueChanged(int value);
    void on_spinBox_5_valueChanged(double arg1);
    void on_slider_6_valueChanged(int value);
    void on_spinBox_6_valueChanged(double arg1);
    void on_slider_7_valueChanged(int value);
    void on_spinBox_7_valueChanged(double arg1);

    void terminal_print_transform();
    void function_move_lin(int i, double time);
    void function_point_to_point(std::array<double, 7> q_goal, double speed);

private slots:

};
#endif // MAINWINDOW_H
