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

#include <argos3/core/simulator/simulator.h>

#include <argos3/core/utility/math/vector2.h>

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>

using namespace argos;

typedef enum
{
    SOFT_TASK = 0,
    HARD_TASK = 1,
} Area_type;

class Area
{
public:
    int id;   // area id (0...15)
    int type; // hard or soft task, i.e. RED or BLUE
    // uint8_t other_type; /* hard or soft task on the other side */

    CVector2 position;                 /* Center of the task */
    double radius;                     /* Radius of the circle to plot */
    argos::CColor color;               /* Color used to represent the area */
    std::vector<int> kilobots_in_area; /* keep counts of how many kbs are in the area*/
    int task_requirement;              /* how many robot are needed to complete the task */
    bool completed;                    /* Flag to understand if the task is accomplished or not */
    double respawn_timer;              /* Time needed to the area to respawn */
    double creation_time;              /* Time at which the area is created/respawned */
    double completed_time;             /* Time at which the area is completed */
    int waiting_timer;                 /* Timer for which kilobots stay on area before leaving*/

    /* constructor */
    Area() : id(-1), type(0), position(CVector2(0, 0)), radius(0) {}

    Area(int id, int type, int other_type, CVector2 position, double radius, int t_requirement) : id(id), type(type),
                                                                                                  position(position), radius(radius), task_requirement(t_requirement)
    {
        this->creation_time = 0.0;
        this->completed = false;
        this->kilobots_in_area.clear();

        this->respawn_timer = 40;

        if (type == HARD_TASK)
        {
            this->color = argos::CColor::RED;
        }
        else
        {
            this->color = argos::CColor::BLUE;
        }

        if (type == HARD_TASK && other_type == HARD_TASK)
            this->waiting_timer = 60;

        else if ((type == SOFT_TASK && other_type == HARD_TASK) ||
                 (type == HARD_TASK && other_type == SOFT_TASK))
            this->waiting_timer = 40;

        else
            this->waiting_timer = 30;
    }

    /* destructor */
    ~Area() {}

    /* check if the point is inside the area */
    bool isInside(CVector2 point, double threshold = 0.0)
    {
        return pow(point.GetX() - position.GetX(), 2) + pow(point.GetY() - position.GetY(), 2) <= (pow(radius - threshold, 2));
    }

    bool isReady()
    {
        return (kilobots_in_area.size() >= task_requirement);
    }

    // check if on top of the area there are the right amount of kilobots, so remove the area from the completable task
    bool isCompleted(int ready)
    {
        if (ready)
            return (kilobots_in_area.size() >= task_requirement);

        return false;
    }

    void set_completed(double kTime)
    {
        this->completed_time = kTime;
        this->completed = true;
    }

    bool Respawn(double kTime)
    {
        if (kTime - this->completed_time > this->respawn_timer)
        {
            this->completed = false;
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
        this->kilobots_in_area.clear();
        this->creation_time = kTime;
        this->completed_time = 0.0;
    }

    bool operator!=(const Area *a)
    {
        return (this->id != a->id || this->completed_time != a->completed_time);
    }

    Area *operator=(const Area *copy)
    {
        if (this == copy)
            return this;

        this->type = copy->type;

        this->color = copy->color;
        this->task_requirement = copy->task_requirement;
        this->respawn_timer = copy->respawn_timer;
        this->waiting_timer = copy->waiting_timer;

        return this;
    }

    void PrintArea()
    {
        std::cout << this->id << '\t' << this->creation_time << '\t' << this->completed_time << '\t' << this->type << '\t' << this->kilobots_in_area.size() << std::endl;
    }
};

#endif // AREA_H
