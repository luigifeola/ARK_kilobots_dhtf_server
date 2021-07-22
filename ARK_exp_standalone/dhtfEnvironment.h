#ifndef DHTFENVIRONMENT_H
#define DHTFENVIRONMENT_H

/**
 * Author: Luigi Feola
 *
 * This is the code that specifies the specific environment used for the DHTF experiment.
 * The environment is divided in a 4x4 grid, of theese 16 cells just ACTIVE_AREAS are activated 
 * that have to be accomplished (occupied) by the kilobots.
 * Colours for the resources are blue and red (which represent respectively a SOTF_TASK or a HARD_TASK, 
 * chainging in the quantity of required kilobots to accomplish the task).
 */
#include <QObject>
#include <QPointF>
#include <QVector>
#include <QVector3D>
#include <QTime>
#include <QMatrix>
#include <QList>
#include <QColor>
#include <QElapsedTimer>

#include <limits>
#include <bitset>

#include <kilobotenvironment.h>
#include "area.h"

#define SCALING 0.5
#define SHIFTX 50 //cnr
#define SHIFTY 50 //cnr
#define ARENA_SIZE 1875
#define ARENA_CENTER ARENA_SIZE/2
#define KILO_DIAMETER 66
#define CM_TO_PIXEL 20

#define ACTIVE_AREAS 4
#define HARD_TASKS_NUMBER 2

typedef enum {
    RANDOM_WALK=0,
    INSIDE_AREA=1,
    LEAVING=2,
    PARTY=3,
}kilobot_state;

class mykilobotenvironment : public KilobotEnvironment
{
    Q_OBJECT
public:
    explicit mykilobotenvironment(QObject *parent = 0);
    void reset();

    QVector<kilobot_state> kilobots_states; // list of all kilobots locations meaning 0 for outside areas, 1 for inside
    QVector<QPointF> kilobots_positions;    // list of all kilobots positions
    QVector<QColor> kilobots_colours;  // list of all kilobots led colours, the led indicate the state of the kilobot
    QVector<bool> kilobots_in_collision; //true at i if kilobot i is in collision

    QVector<Area*> areas;   // list of all areas present in the experiment

    Area* completed_area = new Area(1000, 0, QPointF(1000.0,1000.0), 1.0, 0, 0.0, 0); // random values

    QVector<float> lastSent;    // when the last message was sent to the kb at given position



    float minTimeBetweenTwoMsg; // minimum time between two messages
    double time;
    bool saveLOG;
// signals and slots are used by qt to signal state changes to objects
signals:
    void errorMessage(QString);

public slots:
    void update();
    void updateVirtualSensor(Kilobot kilobot);


private:
    double normAngle(double angle);
    QVector2D VectorRotation2D (double angle, QVector2D vec);
    QVector<int> proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit);
    bool DistantEnoughTasks(QPointF some_position);
};




#endif // DHTFENVIRONMENT_H
