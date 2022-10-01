#ifndef GLOBAL_H
#define GLOBAL_H
#include <QPoint>

//Global arrays.
extern double pointArray[16][16];
extern int gripperArray[16][1];
extern int TORS_Array[16][1];
extern double timeSpeedValueArray[16][1];
extern int motionArray[16][1];

extern std::array<double, 16> points;

extern double jointArray[16][7];

extern double q7;

extern int row;
extern int point_counter;

extern std::array<double, 7> q;

class global
{
public:
    global();
};

#endif // GLOBAL_H
