#include <QtGui>
#include <QtCore>
#include <QMessageBox>
#include <QListWidget>
#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <Eigen/Dense>
#include <functional>
#include <array>
#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "helpwindow.h"
#include "global.h"
#include "button_functions.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include "examples_common.h"
#include "inverse_kinematics.hpp"
#include <chrono>

using namespace std;
using namespace std::chrono;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)

//------------
//Constructor.
{
    ui->setupUi(this);

    //Button pressed moves slider by 1 (signals).
    connect(ui->buttonLeft_1, SIGNAL(clicked()) , this, SLOT(sliderMinus_1())); connect(ui->buttonRight_1, SIGNAL(clicked()) , this, SLOT(sliderPlus_1()));
    connect(ui->buttonLeft_2, SIGNAL(clicked()) , this, SLOT(sliderMinus_2())); connect(ui->buttonRight_2, SIGNAL(clicked()) , this, SLOT(sliderPlus_2()));
    connect(ui->buttonLeft_3, SIGNAL(clicked()) , this, SLOT(sliderMinus_3())); connect(ui->buttonRight_3, SIGNAL(clicked()) , this, SLOT(sliderPlus_3()));
    connect(ui->buttonLeft_4, SIGNAL(clicked()) , this, SLOT(sliderMinus_4())); connect(ui->buttonRight_4, SIGNAL(clicked()) , this, SLOT(sliderPlus_4()));
    connect(ui->buttonLeft_5, SIGNAL(clicked()) , this, SLOT(sliderMinus_5())); connect(ui->buttonRight_5, SIGNAL(clicked()) , this, SLOT(sliderPlus_5()));
    connect(ui->buttonLeft_6, SIGNAL(clicked()) , this, SLOT(sliderMinus_6())); connect(ui->buttonRight_6, SIGNAL(clicked()) , this, SLOT(sliderPlus_6()));
    connect(ui->buttonLeft_7, SIGNAL(clicked()) , this, SLOT(sliderMinus_7())); connect(ui->buttonRight_7, SIGNAL(clicked()) , this, SLOT(sliderPlus_7()));

    terminal_print_transform();

    franka::Robot robot("192.168.40.45");
    setDefaultBehavior(robot);
    franka::RobotState state = robot.readOnce();
    std::array<double, 7> joint_state;
    joint_state = state.q;
    ui->spinBox_1->setValue(joint_state[0]*180/M_PI); ui->spinBox_2->setValue(joint_state[1]*180/M_PI); ui->spinBox_3->setValue(joint_state[2]*180/M_PI);
    ui->spinBox_4->setValue(joint_state[3]*180/M_PI); ui->spinBox_5->setValue(joint_state[4]*180/M_PI); ui->spinBox_6->setValue(joint_state[5]*180/M_PI);
    ui->spinBox_7->setValue(joint_state[6]*180/M_PI);

}

//---------
//Ui close.
MainWindow::~MainWindow()
{delete ui;}

//---------------------------------------------
//Button pressed moves slider by 1 (functions).
void MainWindow::sliderPlus_1()
{ui->slider_1->setValue(ui->slider_1->value()+100);}
void MainWindow::sliderMinus_1()
{ui->slider_1->setValue(ui->slider_1->value()-100);}

void MainWindow::sliderPlus_2()
{ui->slider_2->setValue(ui->slider_2->value()+100);}
void MainWindow::sliderMinus_2()
{ui->slider_2->setValue(ui->slider_2->value()-100);}

void MainWindow::sliderPlus_3()
{ui->slider_3->setValue(ui->slider_3->value()+100);}
void MainWindow::sliderMinus_3()
{ui->slider_3->setValue(ui->slider_3->value()-100);}

void MainWindow::sliderPlus_4()
{ui->slider_4->setValue(ui->slider_4->value()+100);}
void MainWindow::sliderMinus_4()
{ui->slider_4->setValue(ui->slider_4->value()-100);}

void MainWindow::sliderPlus_5()
{ui->slider_5->setValue(ui->slider_5->value()+100);}
void MainWindow::sliderMinus_5()
{ui->slider_5->setValue(ui->slider_5->value()-100);}

void MainWindow::sliderPlus_6()
{ui->slider_6->setValue(ui->slider_6->value()+100);}
void MainWindow::sliderMinus_6()
{ui->slider_6->setValue(ui->slider_6->value()-100);}

void MainWindow::sliderPlus_7()
{ui->slider_7->setValue(ui->slider_7->value()+100);}
void MainWindow::sliderMinus_7()
{ui->slider_7->setValue(ui->slider_7->value()-100);}

//----------------------------
//Help Window pop-up function.
void MainWindow::on_buttonHelp_clicked()
{hWindow = new HelpWindow(this); hWindow->show();}

//------------------------------------------------------------------------------------------------------------------------------
//Slider and spin box conversions (necessary because slider values are strictly integers while we have double value spin boxes).
void MainWindow::on_slider_1_valueChanged(int value)
{double val = (double)value/100;
    ui->spinBox_1->setValue(val);}
void MainWindow::on_spinBox_1_valueChanged(double arg1)
{ui->slider_1->setValue(arg1*100);}

void MainWindow::on_slider_2_valueChanged(int value)
{double val = (double)value/100;
    ui->spinBox_2->setValue(val);}
void MainWindow::on_spinBox_2_valueChanged(double arg1)
{ui->slider_2->setValue(arg1*100);}

void MainWindow::on_slider_3_valueChanged(int value)
{double val = (double)value/100;
    ui->spinBox_3->setValue(val);}
void MainWindow::on_spinBox_3_valueChanged(double arg1)
{ui->slider_3->setValue(arg1*100);}

void MainWindow::on_slider_4_valueChanged(int value)
{double val = (double)value/100;
    ui->spinBox_4->setValue(val);}
void MainWindow::on_spinBox_4_valueChanged(double arg1)
{ui->slider_4->setValue(arg1*100);}

void MainWindow::on_slider_5_valueChanged(int value)
{double val = (double)value/100;
    ui->spinBox_5->setValue(val);}
void MainWindow::on_spinBox_5_valueChanged(double arg1)
{ui->slider_5->setValue(arg1*100);}

void MainWindow::on_slider_6_valueChanged(int value)
{double val = (double)value/100;
    ui->spinBox_6->setValue(val);}
void MainWindow::on_spinBox_6_valueChanged(double arg1)
{ui->slider_6->setValue(arg1*100);}

void MainWindow::on_slider_7_valueChanged(int value)
{double val = (double)value/100;
    ui->spinBox_7->setValue(val);}
void MainWindow::on_spinBox_7_valueChanged(double arg1)
{ui->slider_7->setValue(arg1*100);}

//------------------------------------------------------------------------------
//Function that lists joint poses and transformation matrix in GUI and terminal.
void MainWindow::terminal_print_transform(){
    if (system("CLS")) system("clear");
    franka::Robot robot("192.168.40.45");
    setDefaultBehavior(robot);
    franka::RobotState state = robot.readOnce();
    std::array<double, 16> initial_pose;
    initial_pose = state.O_T_EE_c;
    Eigen::Matrix4d initial_transform=(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    std::array<double, 7> joint_state;
    joint_state = state.q;
    cout<<"-------------------------------------------------------------------------------"<<endl;
    std::cout<<"Current joint positions [rad]:"<<endl<<endl<<joint_state[0]<<", "<<joint_state[1]<<", "<<joint_state[2]<<", "<<joint_state[3]<<", "
            <<joint_state[4]<<", "<<joint_state[5]<<", "<<joint_state[6]<<endl<<endl;
    std::cout<<"Current joint positions [°]:"<<endl<<endl<<
              round((joint_state[0]*180/M_PI)*100)/100<<", "<<
              round((joint_state[1]*180/M_PI)*100)/100<<", "<<
              round((joint_state[2]*180/M_PI)*100)/100<<", "<<
              round((joint_state[3]*180/M_PI)*100)/100<<", "<<
              round((joint_state[4]*180/M_PI)*100)/100<<", "<<
              round((joint_state[5]*180/M_PI)*100)/100<<", "<<
              round((joint_state[6]*180/M_PI)*100)/100<<endl<<endl;
    std::cout<<"Current TCP transformation matrix:"<<endl<<endl<<initial_transform.matrix()<<endl;
    cout<<"-------------------------------------------------------------------------------"<<endl;

    ui->listWidgetCurrent->clear();
    ui->listWidgetCurrent->addItem("X: " + QString::number(round(initial_transform.coeff(0, 3)*100000)/100) + " mm");
    ui->listWidgetCurrent->addItem("Y: " + QString::number(round(initial_transform.coeff(1, 3)*100000)/100) + " mm");
    ui->listWidgetCurrent->addItem("Z: " + QString::number(round(initial_transform.coeff(2, 3)*100000)/100) + " mm");

    ui->listWidgetCurrent->addItem("Ix: " + QString::number(round(initial_transform.coeff(0, 0)*1000)/1000));
    ui->listWidgetCurrent->addItem("Jx: " + QString::number(round(initial_transform.coeff(1, 0)*1000)/1000));
    ui->listWidgetCurrent->addItem("Kx: " + QString::number(round(initial_transform.coeff(2, 0)*1000)/1000));

    ui->listWidgetCurrent->addItem("Iy: " + QString::number(round(initial_transform.coeff(0, 1)*1000)/1000));
    ui->listWidgetCurrent->addItem("Jy: " + QString::number(round(initial_transform.coeff(1, 1)*1000)/1000));
    ui->listWidgetCurrent->addItem("Ky: " + QString::number(round(initial_transform.coeff(2, 1)*1000)/1000));

    ui->listWidgetCurrent->addItem("Iz: " + QString::number(round(initial_transform.coeff(0, 2)*1000)/1000));
    ui->listWidgetCurrent->addItem("Jz: " + QString::number(round(initial_transform.coeff(1, 2)*1000)/1000));
    ui->listWidgetCurrent->addItem("Kz: " + QString::number(round(initial_transform.coeff(2, 2)*1000)/1000));

    ui->spinBox_1->setValue(round((joint_state[0]*180/M_PI)*100)/100); ui->spinBox_2->setValue(round((joint_state[1]*180/M_PI)*100)/100);
    ui->spinBox_3->setValue(round((joint_state[2]*180/M_PI)*100)/100); ui->spinBox_4->setValue(round((joint_state[3]*180/M_PI)*100)/100);
    ui->spinBox_5->setValue(round((joint_state[4]*180/M_PI)*100)/100); ui->spinBox_6->setValue(round((joint_state[5]*180/M_PI)*100)/100);
    ui->spinBox_7->setValue(round((joint_state[6]*180/M_PI)*100)/100);

}

//--------------
//Joint control.
void MainWindow::on_buttonMoveJoints_clicked()
{
    franka::Robot robot("192.168.40.45");
    q[0] = {(ui->spinBox_1->value())*M_PI/180}; q[1] = {(ui->spinBox_2->value())*M_PI/180}; q[2] = {(ui->spinBox_3->value())*M_PI/180};
    q[3] = {(ui->spinBox_4->value())*M_PI/180}; q[4] = {(ui->spinBox_5->value())*M_PI/180}; q[5] = {(ui->spinBox_6->value())*M_PI/180};
    q[6] = {(ui->spinBox_7->value())*M_PI/180};

    try {
      setDefaultBehavior(robot);

      // Set collision behavior.
      robot.setCollisionBehavior(
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

      // Move the robot
      std::array<double, 7> q_goal = {{q[0], q[1], q[2], q[3], q[4], q[5], q[6]}};
      MotionGenerator motion_generator(0.2, q_goal);
      robot.control(motion_generator);
      }
    catch (const franka::Exception& e) {
      std::cout << e.what() << std::endl;
    }
    terminal_print_transform();
}

//-------------
//Gripper open.
void MainWindow::on_buttonOpenGripper_clicked()
{
    franka::Gripper gripper("192.168.40.45");
    double grasping_width = 0.08;
    double speed = 0.5;
    gripper.move(grasping_width, speed);
}

//--------------
//Gripper close.
void MainWindow::on_buttonCloseGripper_clicked()
{
    franka::Gripper gripper("192.168.40.45");
    double grasping_width = 0;
    double speed = 0.5;
    gripper.move(grasping_width, speed);
}

//-----------------------------------------
//Home button moves robot to home position.
void MainWindow::on_buttonHome_clicked()
{
    franka::Robot robot("192.168.40.45");
    try {
      setDefaultBehavior(robot);

      // Set collision behavior.
      robot.setCollisionBehavior(
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

      // Moves robot to Home position.
      std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      MotionGenerator motion_generator(0.2, q_goal);
      robot.control(motion_generator);
      }
    catch (const franka::Exception& e) {
      std::cout << e.what() << std::endl;
    }
    terminal_print_transform();
    ui->spinBox_1->setValue(0); ui->spinBox_2->setValue(-45); ui->spinBox_3->setValue(0); ui->spinBox_4->setValue(-135);
    ui->spinBox_5->setValue(0); ui->spinBox_6->setValue(90); ui->spinBox_7->setValue(45);
}

//-----------------------------
//Resets ui, clears all points.
void MainWindow::on_buttonReset_clicked()
{
    qApp->quit();
    QProcess::startDetached(qApp->arguments()[0], qApp->arguments());
}

//-------------
//Start button.
void MainWindow::on_buttonStart_clicked()
{
    auto start = high_resolution_clock::now();
    for(int i = 0; i < 16; i++){
        if (pointArray[i][0] == 0){
            if(i == 15){
                auto stop = high_resolution_clock::now();
                auto duration = duration_cast<microseconds>(stop - start);
                cout << "Time taken by motion: "
                         << duration.count() << " microseconds" << endl;
                QMessageBox::information(this, "Message", "Motion finished.");
            }
        } else {
            if (gripperArray[i][0] == 1){
                franka::Gripper gripper("192.168.40.45");
                double grasping_width = 0.08;
                double speed = 0.5;
                gripper.move(grasping_width, speed);
            } else if (gripperArray[i][0] == 2){
                franka::Gripper gripper("192.168.40.45");
                double grasping_width = 0;
                double speed = 0.5;
                gripper.move(grasping_width, speed);
            } else if (gripperArray[i][0] == 3){
                franka::Robot robot("192.168.40.45");

                try {
                  setDefaultBehavior(robot);

                  // Set collision behavior.
                  robot.setCollisionBehavior(
                      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                      {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                      {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

                  // Move the robot
                  std::array<double, 7> q_goal = {{jointArray[i][0], jointArray[i][1], jointArray[i][2], jointArray[i][3], jointArray[i][4], jointArray[i][5], jointArray[i][6]}};
                  MotionGenerator motion_generator(0.2, q_goal);
                  robot.control(motion_generator);
                  }
                catch (const franka::Exception& e) {
                  std::cout << e.what() << std::endl;
                }
            }  else {
                if (TORS_Array[i][0] == 1){
                    if(motionArray[i][0] == 1){
                        franka::Robot robot("192.168.40.45");
                        setDefaultBehavior(robot);
                        franka::RobotState state = robot.readOnce();
                        std::array<double, 16> initial_pose;
                        initial_pose = state.O_T_EE;
                        initial_pose[12] = pointArray[i][12]; initial_pose[13] = pointArray[i][13]; initial_pose[14] = pointArray[i][14];

                        std::array<double, 7> diff;
                        std::array<double, 7> joint_state;
                        joint_state = state.q;
                        double q7;

                        if (jointArray[i][0] == 100){
                            q7 = joint_state[6];
                            q[0] = franka_IK_EE_CC(initial_pose, q7, joint_state)[0]; q[1] = franka_IK_EE_CC(initial_pose, q7, joint_state)[1]; q[2] = franka_IK_EE_CC(initial_pose, q7, joint_state)[2];
                            q[3] = franka_IK_EE_CC(initial_pose, q7, joint_state)[3]; q[4] = franka_IK_EE_CC(initial_pose, q7, joint_state)[4]; q[5] = franka_IK_EE_CC(initial_pose, q7, joint_state)[5];
                            q[6] = franka_IK_EE_CC(initial_pose, q7, joint_state)[6];
                            //cout<<q[0]<<endl<<q[1]<<endl<<q[2]<<endl<<q[3]<<endl<<q[4]<<endl<<q[5]<<endl<<q[6]<<endl;
                        } else {
                            q[0] = jointArray[i][0]; q[1] = jointArray[i][1]; q[2] = jointArray[i][2]; q[3] = jointArray[i][3];
                            q[4] = jointArray[i][4]; q[5] = jointArray[i][5]; q[6] = jointArray[i][6];
                        }

                        diff[0] = q[0] - joint_state[0]; diff[1] = q[1] - joint_state[1]; diff[2] = q[2] - joint_state[2]; diff[3] = q[3] - joint_state[3];
                        diff[4] = q[4] - joint_state[4]; diff[5] = q[5] - joint_state[5]; diff[6] = q[6] - joint_state[6];
                        //cout<<diff[0]<<endl<<diff[1]<<endl<<diff[2]<<endl<<diff[3]<<endl<<diff[4]<<endl<<diff[5]<<endl<<diff[6]<<endl;
                        double max = 0; double v;
                        for (int g = 0; g < 7; g++)
                                if (abs(diff[g]) > max)
                                    max = abs(diff[g]);
                        v = (max/timeSpeedValueArray[i][0])*1.45;
                        //cout<<v<<endl;

                        //SIGURNOSNA PETLJA
                        if (v > 1.2){
                            QMessageBox::information(this, "Error", "Motion number " + QString::number(i + 1) + " cannot be executed! Speed or time values are invalid. Exiting motion loop.");
                            break;
                        } else {
                            function_point_to_point(q, v); //RADI
                        }
                        //SIGURNOSNA PETLJA

                    } else {
                        double time = timeSpeedValueArray[i][0];
                        function_move_lin(i, time); //RADI
                    }
                } else {
                    if(motionArray[i][0] == 1){
                        franka::Robot robot("192.168.40.45");
                        setDefaultBehavior(robot);
                        franka::RobotState state = robot.readOnce();
                        std::array<double, 16> initial_pose;
                        initial_pose = state.O_T_EE;
                        initial_pose[12] = pointArray[i][12]; initial_pose[13] = pointArray[i][13]; initial_pose[14] = pointArray[i][14];

                        std::array<double, 7> joint_state;
                        joint_state = state.q;

                        if (jointArray[i][0] == 100){
                            q7 = joint_state[6];
                            q[0] = franka_IK_EE_CC(initial_pose, q7, joint_state)[0]; q[1] = franka_IK_EE_CC(initial_pose, q7, joint_state)[1]; q[2] = franka_IK_EE_CC(initial_pose, q7, joint_state)[2];
                            q[3] = franka_IK_EE_CC(initial_pose, q7, joint_state)[3]; q[4] = franka_IK_EE_CC(initial_pose, q7, joint_state)[4]; q[5] = franka_IK_EE_CC(initial_pose, q7, joint_state)[5];
                            q[6] = franka_IK_EE_CC(initial_pose, q7, joint_state)[6];
                        } else {
                            q[0] = jointArray[i][0]; q[1] = jointArray[i][1]; q[2] = jointArray[i][2]; q[3] = jointArray[i][3];
                            q[4] = jointArray[i][4]; q[5] = jointArray[i][5]; q[6] = jointArray[i][6];
                        }

                        double v = timeSpeedValueArray[i][0];

                        function_point_to_point(q, v); //RADI

                    } else {
                        franka::Robot robot("192.168.40.45");
                        setDefaultBehavior(robot);
                        franka::RobotState state = robot.readOnce();
                        std::array<double, 16> initial_pose;
                        size_t count = 0;
                        initial_pose = state.O_T_EE;
                        initial_pose[12] = pointArray[i][12]; initial_pose[13] = pointArray[i][13]; initial_pose[14] = pointArray[i][14];
                        std::array<double, 7> diff;
                        std::array<double, 7> joint_state;
                        joint_state = state.q;
                        double q7;

                        if (jointArray[i][0] == 100){
                            q7 = joint_state[6];
                            q[0] = franka_IK_EE_CC(initial_pose, q7, joint_state)[0]; q[1] = franka_IK_EE_CC(initial_pose, q7, joint_state)[1]; q[2] = franka_IK_EE_CC(initial_pose, q7, joint_state)[2];
                            q[3] = franka_IK_EE_CC(initial_pose, q7, joint_state)[3]; q[4] = franka_IK_EE_CC(initial_pose, q7, joint_state)[4]; q[5] = franka_IK_EE_CC(initial_pose, q7, joint_state)[5];
                            q[6] = franka_IK_EE_CC(initial_pose, q7, joint_state)[6];
                        } else {
                            q[0] = jointArray[i][0]; q[1] = jointArray[i][1]; q[2] = jointArray[i][2]; q[3] = jointArray[i][3];
                            q[4] = jointArray[i][4]; q[5] = jointArray[i][5]; q[6] = jointArray[i][6];
                        }

                        diff[0] = q[0] - joint_state[0]; diff[1] = q[1] - joint_state[1]; diff[2] = q[2] - joint_state[2]; diff[3] = q[3] - joint_state[3];
                        diff[4] = q[4] - joint_state[4]; diff[5] = q[5] - joint_state[5]; diff[6] = q[6] - joint_state[6];

                        double max = 0; double time;
                        for (int g = 0; g < 7; g++)
                                if (abs(diff[g]) > max)
                                    max = abs(diff[g]);
                        time = max/timeSpeedValueArray[i][0];

                        //SIGURNOSNA PETLJA
                        if ((time < 1 && max > 0.8) || (time < 0.6 && max > 0.48)){
                            QMessageBox::information(this, "Error", "Motion number " + QString::number(i + 1) + " cannot be executed! Speed or time values are invalid. Exiting motion loop.");
                            break;
                        } else {
                            function_move_lin(i, time);
                        }
                        //SIGURNOSNA PETLJA
                    }
                }
            }
        }
    }
    terminal_print_transform();
}

//-----------------------------------------------
//Starts impedance control with given parameters.
void MainWindow::on_buttonImpedance_clicked()
{
    // Compliance parameters
      const double translational_stiffness{ui->spinBoxTransStiff->value()};
      const double rotational_stiffness{ui->spinBoxRotStiff->value()};
      Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
      stiffness.setZero();
      stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
      damping.setZero();
      damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);
      damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                             Eigen::MatrixXd::Identity(3, 3);

      try {
        franka::Robot robot("192.168.40.45");
        setDefaultBehavior(robot);

        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();

        franka::RobotState initial_state = robot.readOnce();

        // equilibrium point is the initial position
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Vector3d position_d(initial_transform.translation());
        Eigen::Quaterniond orientation_d(initial_transform.linear());

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                   {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
            impedance_control_callback = [&](const franka::RobotState& robot_state,
                                             franka::Duration /*duration*/) -> franka::Torques {
          // get state variables
          std::array<double, 7> coriolis_array = model.coriolis(robot_state);
          std::array<double, 42> jacobian_array =
              model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

          // convert to Eigen
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
          Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
          Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
          Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
          Eigen::Vector3d position(transform.translation());
          Eigen::Quaterniond orientation(transform.linear());

          // compute error to desired equilibrium pose
          // position error
          Eigen::Matrix<double, 6, 1> error;
          error.head(3) << position - position_d;

          // orientation error
          // "difference" quaternion
          if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
          }
          // "difference" quaternion
          Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
          error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
          // Transform to base frame
          error.tail(3) << -transform.linear() * error.tail(3);

          // compute control
          Eigen::VectorXd tau_task(7), tau_d(7);

          // Spring damper system with damping ratio=1
          tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
          tau_d << tau_task + coriolis;

          std::array<double, 7> tau_d_array{};
          Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
          return tau_d_array;

        };
        QMessageBox::information(this, "","Cartesian Impedance will start running after clicking OK. After starting try to push the robot and see how it reacts. Push user stop button to end impedance control.");
        robot.control(impedance_control_callback);
      } catch (const franka::Exception& ex) {
        // print exception
        std::cout << ex.what() << std::endl;
      }
}

//-----------------------------------
//Function for point-to-point motion.
void MainWindow::function_point_to_point(std::array<double, 7> q_goal, double speed)
{
    franka::Robot robot("192.168.40.45");
    try {
      setDefaultBehavior(robot);

      // Set collision behavior.
      robot.setCollisionBehavior(
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
          {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

      // Moves robot point-to-point.
      std::array<double, 7> q_goal = {{q[0], q[1], q[2], q[3], q[4], q[5], q[6]}};
      MotionGenerator motion_generator(speed/2, q_goal);
      robot.control(motion_generator);
      }

    catch (const franka::Exception& e) {
      std::cout << e.what() << std::endl;
    }
}

//---------------------------
//Function for linear motion.
void MainWindow::function_move_lin(int i, double time)
    {
        franka::Robot robot("192.168.40.45");
        setDefaultBehavior(robot);
        franka::RobotState robot_state = robot.readOnce();

        double duration = time;
        Eigen::Quaterniond v;
        Eigen::Quaterniond vv;

        Eigen::Affine3d current_pose;
        current_pose=(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));

        Eigen::Affine3d commanded_pose = current_pose;
        commanded_pose(0,3)=pointArray[i][12];
        commanded_pose(1,3)=pointArray[i][13];
        commanded_pose(2,3)=pointArray[i][14];

        if (pointArray[i][0] != 1){
            commanded_pose(0, 0)=pointArray[i][0]; commanded_pose(1, 0)=pointArray[i][1]; commanded_pose(2, 0)=pointArray[i][2];commanded_pose(3, 0)=pointArray[i][3];
            commanded_pose(0, 1)=pointArray[i][4]; commanded_pose(1, 1)=pointArray[i][5]; commanded_pose(2, 1)=pointArray[i][6];commanded_pose(3, 1)=pointArray[i][7];
            commanded_pose(0, 2)=pointArray[i][8]; commanded_pose(1, 2)=pointArray[i][9]; commanded_pose(2, 2)=pointArray[i][10];commanded_pose(3, 2)=pointArray[i][11];
            commanded_pose(3, 3)=pointArray[i][15];
        }

        //For testing purposes.
        /*Eigen::Matrix3d rotZ;
        rotZ<<0,-1,0,1,0,0,0,0,1;
        commanded_pose.linear()= commanded_pose.linear()*rotZ;*/
        //std::cout<<"Commanded pose after rotZ:"<<endl<<endl<<commanded_pose.matrix()<<std::endl<<endl;

        try {
            // Load the kinematics and dynamics model.
                franka::Model model = robot.loadModel();
            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.
                robot.setCollisionBehavior(
                    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

            std::array<double, 16> initial_pose;
            Eigen::Affine3d initial_transform;//(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            double time = 0;
            //Orientation
            Eigen::Affine3d commanded_transform;

              //--------Control loop for Cartesian position and orientation
              //auto cartesian_pose_callback =
                      robot.control([&time, &initial_pose, &initial_transform,&commanded_transform, &commanded_pose, &duration, &v, &vv](const franka::RobotState& robot_state,
                                                 franka::Duration period) -> franka::CartesianPose {
              time += period.toSec();
              //std::cout<<"Time = "<<time<<endl;

              if (time == 0.0) {
                initial_pose = robot_state.O_T_EE_c;
                //cout<<"Initial pose: "<<endl<<Eigen::Matrix4d::Map(initial_pose.data())<<endl;

                //initial_transform=(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
                initial_transform=(Eigen::Matrix4d::Map(robot_state.O_T_EE_c.data()));
                //cout<<"Initial_transform: "<<endl<<initial_transform.matrix()<<endl;

                commanded_transform.matrix()=commanded_pose.matrix(); //if absoulte motion

                v = (initial_transform.linear());
                //cout<<"v ="<<initial_transform.linear()<<endl;
                vv = (commanded_transform.linear());
                //cout<<"vv ="<<commanded_transform.linear()<<endl;

                return franka::CartesianPose(robot_state.O_T_EE_c);
              }

              //Slope - gradual transition function
              double slope = (1 - std::cos(M_PI/ duration * time));//slope goes from 0 to 2
              slope=slope*0.5;//slope goes from 0 to 1

              //Translation
              std::array<double, 16>  new_pose = initial_pose;
              new_pose[12] = initial_pose[12]+slope*(commanded_transform.translation().x()-initial_pose[12]);
              new_pose[13] = initial_pose[13]+slope*(commanded_transform.translation().y()-initial_pose[13]);
              new_pose[14] = initial_pose[14]+slope*(commanded_transform.translation().z()-initial_pose[14]);

              //Slerp - spherical linear interpolation between two 3d rotations
              //Quaterniond result=Slerp(v, vv, slope);//for slerp slope needs to go from 0 to 1
              Eigen::Quaterniond result=v.slerp(slope,vv);
              //std::cout<<"result= "<<result.normalized().coeffs()<<std::endl;
              Eigen::Matrix3d R = result.normalized().toRotationMatrix();
              //std::cout<<"result= "<<R<<std::endl;

              new_pose[0]=R(0,0); new_pose[1]=R(1,0); new_pose[2]=R(2,0);
              new_pose[4]=R(0,1); new_pose[5]=R(1,1); new_pose[6]=R(2,1);
              new_pose[8]=R(0,2); new_pose[9]=R(1,2); new_pose[10]=R(2,2);

              if (time >= duration) {
                return franka::MotionFinished(new_pose);
              }
              return new_pose;
            });

     } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
            robot.automaticErrorRecovery();
          }
}
