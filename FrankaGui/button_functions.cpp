#include "button_functions.h"
#include "ui_mainwindow.h"
#include "mainwindow.h"
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
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include "examples_common.h"

button_functions::button_functions(){}

//-----------------------------------------------------
//This class contains all button functions for the GUI.
//-----------------------------------------------------


//--------------------------
//Add gripper motion button.
void MainWindow::on_buttonAddGripper_clicked()
{
    QString gripper_motion = ui->comboBoxAddGripper->currentText();
    QString stringCompare = " Open";
    if (stringCompare == gripper_motion){
        gripperArray[point_counter][0] = {1}; //Open.
        ui->listWidgetPoint->addItem("Open gripper.");
    } else {
        gripperArray[point_counter][0] = {2}; //Close.
        ui->listWidgetPoint->addItem("Close gripper.");
    }
    pointArray[point_counter][0] = {1};
    motionArray[point_counter][0] = {3};
    TORS_Array[point_counter][0] = {3};
    timeSpeedValueArray[point_counter][0] = {1};
    jointArray[point_counter][0] = {100};
    point_counter++;
}

//------------------------------------------------------------
//Add button adds point values to List box and saves in array.
void MainWindow::on_buttonAddPoint_clicked()
{
    if (point_counter == 16){
        QMessageBox::information(this, "Error", "Maximum nuber of points added.");
    } else {
    double x_value = ui->spinBox_x->value(); double y_value = ui->spinBox_y->value(); double z_value = ui->spinBox_z->value();
    QString motion_value = ui->comboBoxMotion->currentText();
    QString stringCompare = "Linear";

    if(!ui->checkBoxSpeed->isChecked() & !ui->checkBoxTime->isChecked()){
        QMessageBox::information(this, "Error", "Enable at least one option (time or speed).");
    } else if((ui->spinBox_speed->value() == 0.0) & ui->checkBoxSpeed->isChecked()){
        QMessageBox::information(this, "Error", "Speed value cannot be 0.");
    } else {
        if(ui->checkBoxTime->isChecked()){
            ui->listWidgetPoint->addItem("X: " + QString::number(x_value) + "  Y: " + QString::number(y_value) + "  Z: " + QString::number(z_value) + " , " + motion_value + " , " + "Time: " + QString::number(ui->spinBox_time->value()) + " s");
        } else {
            ui->listWidgetPoint->addItem("X: " + QString::number(x_value) + "  Y: " + QString::number(y_value) + "  Z: " + QString::number(z_value) + " , " + motion_value + " , " + "Speed: " + QString::number(ui->spinBox_speed->value()) + " rad/s");
        }

        if (stringCompare == motion_value){
            motionArray[point_counter][0] = {2}; //Linear motion.
        } else {
            motionArray[point_counter][0] = {1}; //Point-to-Point motion.
        }

        if (ui->checkBoxTime->isChecked()){
            TORS_Array[point_counter][0] = {1}; //Time.
            timeSpeedValueArray[point_counter][0] = {ui->spinBox_time->value()};
        } else {
            TORS_Array[point_counter][0] = {2}; //Speed.
            timeSpeedValueArray[point_counter][0] = {ui->spinBox_speed->value()};
        }
        pointArray[point_counter][0] = {1};
        pointArray[point_counter][12] = {x_value/1000};
        pointArray[point_counter][13] = {y_value/1000};
        pointArray[point_counter][14] = {z_value/1000};
        gripperArray[point_counter][0] = {4}; //Open (1), Close(2), Joint motion(3), PTP/Linear motion(4).
        jointArray[point_counter][0] = {100};
        point_counter++;
        }
    }
}

//-----------------------------------------------------------------------------------------
//Add current button adds current TCP position point values to List box and saves in array.
void MainWindow::on_buttonAddCurrent_clicked()
{
    if (point_counter == 16){
        QMessageBox::information(this, "Error", "Maximum nuber of points added.");
    } else {
    QString motion_value = ui->comboBoxMotion->currentText();
    QString stringCompare = "Linear";

    if(!ui->checkBoxSpeed->isChecked() & !ui->checkBoxTime->isChecked()){
        QMessageBox::information(this, "Error", "Enable at least one option (time or speed).");
    } else if((ui->spinBox_speed->value() == 0.0) & ui->checkBoxSpeed->isChecked()){
        QMessageBox::information(this, "Error", "Speed value cannot be 0.");
    } else{

        franka::Robot robot("192.168.40.45");
              setDefaultBehavior(robot);
              franka::RobotState robot_state = robot.readOnce();
              pointArray[point_counter][0] = {robot_state.O_T_EE.at(0)}; pointArray[point_counter][1] = {robot_state.O_T_EE.at(1)}; pointArray[point_counter][2] = {robot_state.O_T_EE.at(2)};
              pointArray[point_counter][3] = {robot_state.O_T_EE.at(3)}; pointArray[point_counter][4] = {robot_state.O_T_EE.at(4)}; pointArray[point_counter][5] = {robot_state.O_T_EE.at(5)};
              pointArray[point_counter][6] = {robot_state.O_T_EE.at(6)}; pointArray[point_counter][7] = {robot_state.O_T_EE.at(7)}; pointArray[point_counter][8] = {robot_state.O_T_EE.at(8)};
              pointArray[point_counter][9] = {robot_state.O_T_EE.at(9)}; pointArray[point_counter][10] = {robot_state.O_T_EE.at(10)}; pointArray[point_counter][11] = {robot_state.O_T_EE.at(11)};
              pointArray[point_counter][12] = {robot_state.O_T_EE.at(12)}; pointArray[point_counter][13] = {robot_state.O_T_EE.at(13)}; pointArray[point_counter][14] = {robot_state.O_T_EE.at(14)};
              pointArray[point_counter][15] = {robot_state.O_T_EE.at(15)};

        gripperArray[point_counter][0] = {4}; //Open (1), Close(2), Joint motion(3), PTP/Linear motion(4).

        if(ui->checkBoxTime->isChecked()){
            ui->listWidgetPoint->addItem("X: " + QString::number(round(pointArray[point_counter][12]*100000)/100) + "  Y: " + QString::number(round(pointArray[point_counter][13]*100000)/100) + "  Z: " + QString::number(round(pointArray[point_counter][14]*100000)/100) + " , " + motion_value + " , " + "Time: " + QString::number(ui->spinBox_time->value()) + " s");
        } else{
            ui->listWidgetPoint->addItem("X: " + QString::number(round(pointArray[point_counter][12]*100000)/100) + "  Y: " + QString::number(round(pointArray[point_counter][13]*100000)/100) + "  Z: " + QString::number(round(pointArray[point_counter][14]*100000)/100) + " , " + motion_value + " , " + "Speed: " + QString::number(ui->spinBox_speed->value()) + " rad/s");
        }

        if (stringCompare == motion_value){
            motionArray[point_counter][0] = {2}; //Linear motion.
        } else {
            motionArray[point_counter][0] = {1}; //Point-to-Point motion.
        }

        if (ui->checkBoxTime->isChecked()){
            TORS_Array[point_counter][0] = {1}; //Time.
            timeSpeedValueArray[point_counter][0] = {ui->spinBox_time->value()};
        } else {
            TORS_Array[point_counter][0] = {2}; //Speed.
            timeSpeedValueArray[point_counter][0] = {ui->spinBox_speed->value()};
        }
        std::array<double, 7> joint_state;
        joint_state = robot_state.q;
        jointArray[point_counter][0] = joint_state[0]; jointArray[point_counter][1] = joint_state[1]; jointArray[point_counter][2] = joint_state[2];
        jointArray[point_counter][3] = joint_state[3]; jointArray[point_counter][4] = joint_state[4]; jointArray[point_counter][5] = joint_state[5];
        jointArray[point_counter][6] = joint_state[6];
        point_counter++;
        }
    }
}

//--------------
//Delete button.
void MainWindow::on_buttonDelete_clicked()
{
    for(QListWidgetItem *item: ui->listWidgetPoint->selectedItems()){ //Gets int value of row selected.
        row = ui->listWidgetPoint->row(item);
        qDebug() << row;
    }

    QListWidgetItem *it = ui->listWidgetPoint->takeItem(ui->listWidgetPoint->currentRow()); //Deletes currently selected row.
    delete it;

    int no_of_columns = 16; int no_of_rows = 16;
    point_counter--;
    gripperArray[row][0] = {0};
    TORS_Array[row][0] = {0};
    timeSpeedValueArray[row][0] = {0};
    motionArray[row][0] = {0};
    pointArray[row][0] = {0}; //Changes value in deleted row to 0 in array index [row][1].
    jointArray[row][0] = {0};

    int i, j, k;
    for (i = 0 ; i < no_of_rows ; i++) { //Deletes row {0, x2, x3, x4, x5} in array and moves all points up by one step.
            if (pointArray[i][0] == 0) {
                for (k = i ; k < no_of_rows - 1 ; k++) {
                    for (j = 0 ; j < no_of_columns ; j++) {
                        pointArray[k][j] = pointArray[k+1][j];
                    }
                }
               i--;
               no_of_rows--;
            }
    }

    int no_of_columns2 = 7;
    int no_of_rows2 = 16;
    for (i = 0 ; i < no_of_rows2 ; i++) { //Deletes row {0, x2, x3, x4, x5} in array and moves all points up by one step.
            if (jointArray[i][0] == 0) {
                for (k = i ; k < no_of_rows2 - 1 ; k++) {
                    for (j = 0 ; j < no_of_columns2 ; j++) {
                        jointArray[k][j] = jointArray[k+1][j];
                    }
                }
               i--;
               no_of_rows2--;
            }
    }

    for (int i = row; i < 16; i++){
        gripperArray[i][0] = gripperArray[i+1][0];
        TORS_Array[i][0] = TORS_Array[i+1][0];
        timeSpeedValueArray[i][0] = timeSpeedValueArray[i+1][0];
        motionArray[i][0] = motionArray[i+1][0];
    }   
}

//--------------
//Change button.
void MainWindow::on_buttonChange_clicked()
{
    for(QListWidgetItem *item: ui->listWidgetPoint->selectedItems()){ //Gets int value of row selected.
        row = ui->listWidgetPoint->row(item);
        qDebug() << row;
    }

    if(gripperArray[row][0] == 1 || gripperArray[row][0] == 2 || gripperArray[row][0] == 3){
         QMessageBox::information(this, "Error", "You cannot change this point.");
    } else {

    double x_value = ui->spinBox_x->value(); double y_value = ui->spinBox_y->value(); double z_value = ui->spinBox_z->value();
    QString motion_value = ui->comboBoxMotion->currentText();
    QString stringCompare = "Linear";

    if(!ui->checkBoxSpeed->isChecked() & !ui->checkBoxTime->isChecked()){
        QMessageBox::information(this, "Error", "Enable at least one option (time or speed).");
    } else if((ui->spinBox_speed->value() == 0.0) & ui->checkBoxSpeed->isChecked()){
        QMessageBox::information(this, "Error", "Speed value cannot be 0.");
    } else{
        QListWidgetItem *it = ui->listWidgetPoint->takeItem(ui->listWidgetPoint->currentRow()); //Deletes currently selected row.
        delete it;
        if(ui->checkBoxTime->isChecked()){
            QListWidgetItem *newPoint = new QListWidgetItem;
            newPoint->setText("X: " + QString::number(x_value) + "  Y: " + QString::number(y_value) + "  Z: " + QString::number(z_value) + " , " + motion_value + " , " + "Time: " + QString::number(ui->spinBox_time->value()) + " s");
            ui->listWidgetPoint->insertItem(row, newPoint);
        } else{
            QListWidgetItem *newPoint = new QListWidgetItem;
            newPoint->setText("X: " + QString::number(x_value) + "  Y: " + QString::number(y_value) + "  Z: " + QString::number(z_value) + " , " + motion_value + " , " + "Speed: " + QString::number(ui->spinBox_speed->value()) + " rad/s");
            ui->listWidgetPoint->insertItem(row, newPoint);
        }

        if (stringCompare == motion_value){
            motionArray[row][0] = {2}; //Linear motion.
        } else {
            motionArray[row][0] = {1}; //Point-to-Point motion.
        }

        if (ui->checkBoxTime->isChecked()){
            TORS_Array[row][0] = {1}; //Time.
            timeSpeedValueArray[row][0] = {ui->spinBox_time->value()};
        } else {
            TORS_Array[row][0] = {2}; //Speed.
            timeSpeedValueArray[row][0] = {ui->spinBox_speed->value()};
        }
        pointArray[row][0] = {1};
        pointArray[row][12] = {x_value/1000};
        pointArray[row][13] = {y_value/1000};
        pointArray[row][14] = {z_value/1000};
        gripperArray[row][0] = {4}; //Open (1), Close(2), Joint motion(3), PTP/Linear motion(4).
        jointArray[point_counter][0] = {100};
        }
    }
}

//-----------------
//Move down button.
void MainWindow::on_buttonMoveDown_clicked()
{
    for(QListWidgetItem *item: ui->listWidgetPoint->selectedItems()){ //Gets int value of row selected.
        row = ui->listWidgetPoint->row(item);
        qDebug() << row;
    }

    int moveDownA = row;
    int moveDownB = row + 1;
    int exp = ui->listWidgetPoint->model()->rowCount();
    if(moveDownB == exp){
        QMessageBox::information(this, "Error", "Point cannot be moved down.");
    } else{
    double temp = 0.0;
        for(int i = 0; i < 16; i++){  //Swaps array positions.
            temp = pointArray[moveDownA][i];
            pointArray[moveDownA][i] = pointArray[moveDownB][i];
            pointArray[moveDownB][i] = temp;
        }

        for(int i = 0; i < 7; i++){  //Swaps array positions.
            temp = jointArray[moveDownA][i];
            jointArray[moveDownA][i] = jointArray[moveDownB][i];
            jointArray[moveDownB][i] = temp;
        }

        temp = gripperArray[moveDownA][0];
        gripperArray[moveDownA][0] = gripperArray[moveDownB][0];
        gripperArray[moveDownB][0] = temp;

        temp = TORS_Array[moveDownA][0];
        TORS_Array[moveDownA][0] = TORS_Array[moveDownB][0];
        TORS_Array[moveDownB][0] = temp;

        temp = timeSpeedValueArray[moveDownA][0];
        timeSpeedValueArray[moveDownA][0] = timeSpeedValueArray[moveDownB][0];
        timeSpeedValueArray[moveDownB][0] = temp;

        temp = motionArray[moveDownA][0];
        motionArray[moveDownA][0] = motionArray[moveDownB][0];
        motionArray[moveDownB][0] = temp;

        int swap = ui->listWidgetPoint->currentRow();
        if (swap >= 0) { //Currently selected.
            auto current_item = ui->listWidgetPoint->currentItem();
            auto previous_item = ui->listWidgetPoint->item(swap+1);
            std::swap(*current_item,*previous_item);
            ui->listWidgetPoint->repaint(); // Force repaint, else will not show change.
        }
        ui->listWidgetPoint->doItemsLayout();
    }
}

//---------------
//Move up button.
void MainWindow::on_buttonMoveUp_clicked()
{
    for(QListWidgetItem *item: ui->listWidgetPoint->selectedItems()){ //Gets int value of row selected.
        row = ui->listWidgetPoint->row(item);
        qDebug() << row;
    }

    int moveUpA = row;
    int moveUpB = row - 1;
    if(moveUpB == -1){
        QMessageBox::information(this, "Error", "Point cannot be moved up.");
    } else{
    double temp2 = 0.0;
        for(int i = 0; i < 16; i++){  //Swaps array positions.
            temp2 = pointArray[moveUpA][i];
            pointArray[moveUpA][i] = pointArray[moveUpB][i];
            pointArray[moveUpB][i] = temp2;
        }

        for(int i = 0; i < 7; i++){  //Swaps array positions.
            temp2 = jointArray[moveUpA][i];
            jointArray[moveUpA][i] = jointArray[moveUpB][i];
            jointArray[moveUpB][i] = temp2;
        }

        temp2 = gripperArray[moveUpA][0];
        gripperArray[moveUpA][0] = gripperArray[moveUpB][0];
        gripperArray[moveUpB][0] = temp2;

        temp2 = TORS_Array[moveUpA][0];
        TORS_Array[moveUpA][0] = TORS_Array[moveUpB][0];
        TORS_Array[moveUpB][0] = temp2;

        temp2 = timeSpeedValueArray[moveUpA][0];
        timeSpeedValueArray[moveUpA][0] = timeSpeedValueArray[moveUpB][0];
        timeSpeedValueArray[moveUpB][0] = temp2;

        temp2 = motionArray[moveUpA][0];
        motionArray[moveUpA][0] = motionArray[moveUpB][0];
        motionArray[moveUpB][0] = temp2;

        int swap2 = ui->listWidgetPoint->currentRow();
        if (swap2 >= 0)  //Currently selected.
        {
            auto current_item = ui->listWidgetPoint->currentItem();
            auto previous_item = ui->listWidgetPoint->item(swap2-1);
            std::swap(*current_item,*previous_item);
            ui->listWidgetPoint->repaint(); // Force repaint, else will not show change.
        }
        ui->listWidgetPoint->doItemsLayout();
    }
}

//------------------------------
//Get joints to spin box button.
void MainWindow::on_buttonGetJoints_clicked()
{
    franka::Robot robot("192.168.40.45");
    setDefaultBehavior(robot);
    franka::RobotState state = robot.readOnce();
    std::array<double, 7> joint_state;
    joint_state = state.q;
    ui->spinBox_1->setValue(round((joint_state[0]*180/M_PI)*100)/100); ui->spinBox_2->setValue(round((joint_state[1]*180/M_PI)*100)/100);
    ui->spinBox_3->setValue(round((joint_state[2]*180/M_PI)*100)/100); ui->spinBox_4->setValue(round((joint_state[3]*180/M_PI)*100)/100);
    ui->spinBox_5->setValue(round((joint_state[4]*180/M_PI)*100)/100); ui->spinBox_6->setValue(round((joint_state[5]*180/M_PI)*100)/100);
    ui->spinBox_7->setValue(round((joint_state[6]*180/M_PI)*100)/100);
}

//------------------------
//Add joint motion button.
void MainWindow::on_buttonAddJointMotion_clicked()
{
    ui->listWidgetPoint->addItem("Joint motion: "
            + QString::number(ui->spinBox_1->value()) + "°, " + QString::number(ui->spinBox_2->value()) + "°, "
            + QString::number(ui->spinBox_3->value()) + "°, " + QString::number(ui->spinBox_4->value()) + "°, "
            + QString::number(ui->spinBox_5->value()) + "°, " + QString::number(ui->spinBox_6->value()) + "°, "
            + QString::number(ui->spinBox_7->value()) + "°, ");

    if (ui->spinBox_1->value() == 0){
        jointArray[point_counter][0] = {0.01*M_PI/180};
    } else {
        jointArray[point_counter][0] = {ui->spinBox_1->value()*M_PI/180};
    }

    jointArray[point_counter][1] = {ui->spinBox_2->value()*M_PI/180}; jointArray[point_counter][2] = {ui->spinBox_3->value()*M_PI/180};
    jointArray[point_counter][3] = {ui->spinBox_4->value()*M_PI/180}; jointArray[point_counter][4] = {ui->spinBox_5->value()*M_PI/180};
    jointArray[point_counter][5] = {ui->spinBox_6->value()*M_PI/180}; jointArray[point_counter][6] = {ui->spinBox_7->value()*M_PI/180};

    gripperArray[point_counter][0] = {3}; //Joint motion.

    motionArray[point_counter][0] = {3};
    TORS_Array[point_counter][0] = {3};
    timeSpeedValueArray[point_counter][0] = {1};
    pointArray[point_counter][0] = {100};
    point_counter++;
}
