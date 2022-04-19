/**
 * Custom definition of an area
 * With the term area we refer to the single area (e.g. the single coloured circle)
 * Areas disasppears when the task on that area is accomplished
 *
 * @author Luigi Feola
 * @email luigi.feola@istc.cnr.it
 */

#ifndef AREA_H
#define AREA_H

#include <math.h>
#include <stdlib.h>

#include <QPointF>
#include <QColor>
#include <QVector>
#include <QDebug>

#define SOFT_TASK_COMPLETED 2
#define HARD_TASK_COMPLETED 4

typedef enum
{
    SOFT_TASK=0,
    HARD_TASK=1,
} Area_type;

class Area
{
public:
    uint id; // area id (0...15)
    uint8_t type; //hard or soft task, i.e. RED or BLUE

    QPointF position; /* Center of the task */
    double radius; /* Radius of the circle to plot */
    QColor color; /* Color used to represent the area */
    QVector<uint> kilobots_in_area; /* keep counts of how many kbs are in the area*/
    bool completed;  /* Flag to understand if the task is accomplished or not */
    bool party_message_sent; /* Flag to check if kilobots_in_area received the area completion message */
    double respawn_timer; /* Time needed to the area to respawn */
    double creation_time; /* Time at which the area is created/respawned */
    double completed_time; /* Time at which the area is completed */

    int waiting_timer; /* Timer for which kilobots stay on area before leaving*/

    /* constructor */
    Area() : id(-1), type(0), position(QPointF(0,0)), radius(0) {}

    Area(uint id, uint8_t type, uint8_t other_type, QPointF position, double radius) :
        id(id), type(type), position(position), radius(radius)
    {
        this->creation_time = 0.0;
        this->completed = false;
        this->party_message_sent = false;
        this->kilobots_in_area.clear();

        this->respawn_timer = 40;

        if(type == HARD_TASK)
        {
            this->color = Qt::red;
        }
        else
        {
            this->color = Qt::blue;
        }

        if (type == HARD_TASK && other_type == HARD_TASK)
            this->waiting_timer = 60;

        else if( (type == SOFT_TASK && other_type == HARD_TASK) ||
                 (type == HARD_TASK && other_type == SOFT_TASK) )
            this->waiting_timer = 40;

        else
            this->waiting_timer = 30;
            
    }

    Area(const Area & a1) : kilobots_in_area(a1.kilobots_in_area)
    {
        this->id = a1.id;
        this->type = a1.type;
        this->creation_time = a1.creation_time;
        this->completed_time = a1.completed_time;
    }


    /* destructor */
    ~Area(){}

    /* check if the point is inside the area */
    bool isInside(QPointF point, double threshold = 0.0) {
       return pow(point.x()-position.x(),2)+pow(point.y()-position.y(),2) <= (pow(radius-threshold,2));
    }

    bool isReady()
    {
       return ( (this->type == HARD_TASK && kilobots_in_area.size() >= HARD_TASK_COMPLETED) ||
                (this->type == SOFT_TASK && kilobots_in_area.size() >= SOFT_TASK_COMPLETED) );
    }

    // check if on top of the area there are the right amount of kilobots, so remove the area from the completable task
    bool isCompleted(double kTime, int ready)
    {
        if(ready)
        {
            if( (this->type == HARD_TASK && kilobots_in_area.size() >= HARD_TASK_COMPLETED) ||
                (this->type == SOFT_TASK && kilobots_in_area.size() >= SOFT_TASK_COMPLETED))
            {
                this->completed_time = kTime;
                this->completed = true;
            }
        }

        return this->completed;
    }

    void set_completed(double kTime, Area* a_copy)
    {
        this->completed_time = kTime;
        *a_copy = *this;
        this->completed = true;
    }


    bool Respawn(double kTime)
    {
        if(kTime - this->completed_time > this->respawn_timer)
        {
            this->completed = false;
            this->party_message_sent = false;
            this->kilobots_in_area.clear();
            this->creation_time = kTime;
            this->completed_time = 0.0;
            return true;
        }
        else
            return false;
    }

    void received_Respawn(double kTime)
    {
        this->completed = false;
        this->party_message_sent = false;
        this->kilobots_in_area.clear();
        this->creation_time = kTime;
        this->completed_time = 0.0;
    }

    bool operator !=(const Area* a)
    {
        return (this->id != a->id || this->completed_time != a->completed_time);
    }

    void PrintArea()
    {
        qDebug() <<
                    this->id << '\t' <<
                    this->creation_time << '\t' <<
                    this->completed_time << '\t' <<
                    this->type << '\t' <<
                    this->kilobots_in_area.size();
    }
};

#endif // AREA_H
