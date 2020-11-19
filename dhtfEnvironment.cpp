#ifndef DHTFENVIRONMENT_CPP
#define DHTFENVIRONMENT_CPP

#include "dhtfEnvironment.h"
#include "area.h"

#include "kilobot.h"

#include <math.h>
#include <stdlib.h>
#include <random>

#include <QVector>
#include <QVector2D>
#include <QLineF>
#include <QDebug>
#include <QtMath>
#include <QColor>


mykilobotenvironment::mykilobotenvironment(QObject *parent) : KilobotEnvironment(parent) {
    // environment specifications
    this->ArenaX = 0.45;
    this->ArenaY = 0.45;
    this->ongoingRuntimeIdentification = false;

    this->saveLOG = false;

    this->initialised_client = false;
    this->initialise_buffer = "";
    this->send_buffer = "";
    this->receive_buffer= "";
    // define environment:
    // call any functions to setup features in the environment (goals, homes locations and parameters).
    reset();
}

void mykilobotenvironment::initialiseEnvironment(QVector<int> activated_areas, QVector<uint> hard_tasks, QVector<uint> hard_tasks_client){

    double white_space = SCALING * 120.0;
    double radius = (((2.0*ARENA_CENTER - white_space)/ 4.0) - white_space)/2.0;
    // qDebug() << QString("radius") << radius;

    if(std::find(activated_areas.begin(),activated_areas.end(), 0) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 0) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 0) != hard_tasks_client.end())
                areas.push_back(new Area(0, HARD_TASK, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(0, HARD_TASK, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,1.0*radius + 1.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 0) != hard_tasks_client.end())
                areas.push_back(new Area(0, SOFT_TASK, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(0, SOFT_TASK, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,1.0*radius + 1.0*white_space), radius));
        }

    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 1) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 1) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 1) != hard_tasks_client.end())
                areas.push_back(new Area(1, HARD_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(1, HARD_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,1.0*radius + 1.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 1) != hard_tasks_client.end())
                areas.push_back(new Area(1, SOFT_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(1, SOFT_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,1.0*radius + 1.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 2) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 2) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 2) != hard_tasks_client.end())
                areas.push_back(new Area(2, HARD_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(2, HARD_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,1.0*radius + 1.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 2) != hard_tasks_client.end())
                areas.push_back(new Area(2, SOFT_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(2, SOFT_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,1.0*radius + 1.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 3) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 3) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 3) != hard_tasks_client.end())
                areas.push_back(new Area(3, HARD_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(3, HARD_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,1.0*radius + 1.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 3) != hard_tasks_client.end())
                areas.push_back(new Area(3, SOFT_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(3, SOFT_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,1.0*radius + 1.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 4) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 4) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 4) != hard_tasks_client.end())
                areas.push_back(new Area(4, HARD_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,1.0*radius + 1.0*white_space), radius));
            else
                areas.push_back(new Area(4, HARD_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,1.0*radius + 1.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 4) != hard_tasks_client.end())
                areas.push_back(new Area(4, SOFT_TASK, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,3.0*radius + 2.0*white_space), radius));
            else
                areas.push_back(new Area(4, SOFT_TASK, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,3.0*radius + 2.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 5) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 5) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 5) != hard_tasks_client.end())
                areas.push_back(new Area(5, HARD_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,3.0*radius + 2.0*white_space), radius));
            else
                areas.push_back(new Area(5, HARD_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,3.0*radius + 2.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 5) != hard_tasks_client.end())
                areas.push_back(new Area(5, SOFT_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,3.0*radius + 2.0*white_space), radius));
            else
                areas.push_back(new Area(5, SOFT_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,3.0*radius + 2.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 6) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 6) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 6) != hard_tasks_client.end())
                areas.push_back(new Area(6, HARD_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,3.0*radius + 2.0*white_space), radius));
            else
                areas.push_back(new Area(6, HARD_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,3.0*radius + 2.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 6) != hard_tasks_client.end())
                areas.push_back(new Area(6, SOFT_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,3.0*radius + 2.0*white_space), radius));
            else
                areas.push_back(new Area(6, SOFT_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,3.0*radius + 2.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 7) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 7) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 7) != hard_tasks_client.end())
                areas.push_back(new Area(7, HARD_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,3.0*radius + 2.0*white_space), radius));
            else
                areas.push_back(new Area(7, HARD_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,3.0*radius + 2.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 7) != hard_tasks_client.end())
                areas.push_back(new Area(7, SOFT_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,3.0*radius + 2.0*white_space), radius));
            else
                areas.push_back(new Area(7, SOFT_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,3.0*radius + 2.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 8) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 8) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 8) != hard_tasks_client.end())
                areas.push_back(new Area(8, HARD_TASK, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(8, HARD_TASK, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,5.0*radius + 3.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 8) != hard_tasks_client.end())
                areas.push_back(new Area(8, SOFT_TASK, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(8, SOFT_TASK, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,5.0*radius + 3.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 9) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 9) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 9) != hard_tasks_client.end())
                areas.push_back(new Area(9, HARD_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(9, HARD_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,5.0*radius + 3.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 9) != hard_tasks_client.end())
                areas.push_back(new Area(9, SOFT_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(9, SOFT_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,5.0*radius + 3.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 10) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 10) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 10) != hard_tasks_client.end())
                areas.push_back(new Area(10, HARD_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(10, HARD_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,5.0*radius + 3.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 10) != hard_tasks_client.end())
                areas.push_back(new Area(10, SOFT_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(10, SOFT_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,5.0*radius + 3.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 11) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 11) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 11) != hard_tasks_client.end())
                areas.push_back(new Area(11, HARD_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(11, HARD_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,5.0*radius + 3.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 11) != hard_tasks_client.end())
                areas.push_back(new Area(11, SOFT_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,5.0*radius + 3.0*white_space), radius));
            else
                areas.push_back(new Area(11, SOFT_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,5.0*radius + 3.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 12) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 12) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 12) != hard_tasks_client.end())
                areas.push_back(new Area(12, HARD_TASK, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(12, HARD_TASK, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,7.0*radius + 4.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 12) != hard_tasks_client.end())
                areas.push_back(new Area(12, SOFT_TASK, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(12, SOFT_TASK, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,7.0*radius + 4.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 13) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 13) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 13) != hard_tasks_client.end())
                areas.push_back(new Area(13, HARD_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(13, HARD_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,7.0*radius + 4.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 13) != hard_tasks_client.end())
                areas.push_back(new Area(13, SOFT_TASK, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(13, SOFT_TASK, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,7.0*radius + 4.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 14) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 14) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 14) != hard_tasks_client.end())
                areas.push_back(new Area(14, HARD_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(14, HARD_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,7.0*radius + 4.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 14) != hard_tasks_client.end())
                areas.push_back(new Area(14, SOFT_TASK, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(14, SOFT_TASK, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,7.0*radius + 4.0*white_space), radius));
        }
    }

    if(std::find(activated_areas.begin(),activated_areas.end(), 15) != activated_areas.end())
    {
        if(std::find(hard_tasks.begin(),hard_tasks.end(), 15) != hard_tasks.end())
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 15) != hard_tasks_client.end())
                areas.push_back(new Area(15, HARD_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(15, HARD_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,7.0*radius + 4.0*white_space), radius));
        }

        else
        {
            if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), 15) != hard_tasks_client.end())
                areas.push_back(new Area(15, SOFT_TASK, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,7.0*radius + 4.0*white_space), radius));
            else
                areas.push_back(new Area(15, SOFT_TASK, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,7.0*radius + 4.0*white_space), radius));
        }
    }


   // areas.push_back(new Area(0, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,1.0*radius + 1.0*white_space), radius));
   // areas.push_back(new Area(1, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,1.0*radius + 1.0*white_space), radius));
   // areas.push_back(new Area(2, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,1.0*radius + 1.0*white_space), radius));
   // areas.push_back(new Area(3, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,1.0*radius + 1.0*white_space), radius));

   // areas.push_back(new Area(4, SOFT_TASK, QPointF(1.0*radius + 1.0*white_space,3.0*radius + 2.0*white_space), radius));
   // areas.push_back(new Area(5, SOFT_TASK, QPointF(3.0*radius + 2.0*white_space,3.0*radius + 2.0*white_space), radius));
   // areas.push_back(new Area(6, SOFT_TASK, QPointF(5.0*radius + 3.0*white_space,3.0*radius + 2.0*white_space), radius));
   // areas.push_back(new Area(7, SOFT_TASK, QPointF(7.0*radius + 4.0*white_space,3.0*radius + 2.0*white_space), radius));

   // areas.push_back(new Area(8, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,5.0*radius + 3.0*white_space), radius));
   // areas.push_back(new Area(9, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,5.0*radius + 3.0*white_space), radius));
   // areas.push_back(new Area(10, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,5.0*radius + 3.0*white_space), radius));
   // areas.push_back(new Area(11, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,5.0*radius + 3.0*white_space), radius));

   // areas.push_back(new Area(12, HARD_TASK, QPointF(1.0*radius + 1.0*white_space,7.0*radius + 4.0*white_space), radius));
   // areas.push_back(new Area(13, HARD_TASK, QPointF(3.0*radius + 2.0*white_space,7.0*radius + 4.0*white_space), radius));
   // areas.push_back(new Area(14, HARD_TASK, QPointF(5.0*radius + 3.0*white_space,7.0*radius + 4.0*white_space), radius));
   // areas.push_back(new Area(15, HARD_TASK, QPointF(7.0*radius + 4.0*white_space,7.0*radius + 4.0*white_space), radius));

}

void mykilobotenvironment::reset(){
    this->time = 0;
    this->minTimeBetweenTwoMsg = 0;
    this->ongoingRuntimeIdentification = false;

    areas.clear();
    kilobots_states.clear();
    kilobots_states_LOG.clear();


    kilobots_positions.clear();
    kilobots_colours.clear();

    std::default_random_engine re;
    re.seed(0);
//    re.seed(qrand());
    QVector<int> activated_areas;
    QVector<uint> hard_tasks;
    QVector<uint> hard_tasks_client;

    int start = 0;
    int end = 15;
    while(activated_areas.size()<ACTIVE_AREAS)
    {
        if(ACTIVE_AREAS-1 > end){
            qDebug() << " Requested more areas than the available ones, you should increase end";
        }
        std::uniform_int_distribution<uint> distr(start, end);
        uint random_number;
        do{
        // qDebug() << QString("Drawing a random number");
            random_number = distr(re);
        }while (std::find(activated_areas.begin(),activated_areas.end(), random_number) != activated_areas.end());
        activated_areas.push_back(random_number);
    }
    std::sort(activated_areas.begin(), activated_areas.end());

    // Show selected areas
    qDebug() << QString("Selected areas");
    //Print selected areas
    for(int act_ar : activated_areas)
    {
        qDebug() << act_ar;
    }


    while(hard_tasks.size()<HARD_TASKS_NUMBER)
    {
        std::uniform_int_distribution<uint> distr(start, end);
        uint random_number;
        do{
            random_number = distr(re);
        // qDebug() << QString("Drawing the number: ") << random_number;
        }while (std::find(activated_areas.begin(),activated_areas.end(), random_number) == activated_areas.end() ||
                std::find(hard_tasks.begin(),hard_tasks.end(), random_number) != hard_tasks.end());
        hard_tasks.push_back(random_number);
    }
    std::sort(hard_tasks.begin(), hard_tasks.end());

    // Show selected hard tasts for server
    qDebug() << QString("Selected hard tasks server");
    //Print selected areas
    for(uint h_task : hard_tasks)
    {
        qDebug() << h_task;
    }


    while(hard_tasks_client.size()<HARD_TASKS_NUMBER)
    {
        std::uniform_int_distribution<uint> distr(start, end);
        uint random_number;
        do{
            random_number = distr(re);
        // qDebug() << QString("Drawing the number: ") << random_number;
        }while (std::find(activated_areas.begin(),activated_areas.end(), random_number) == activated_areas.end() ||
                std::find(hard_tasks_client.begin(),hard_tasks_client.end(), random_number) != hard_tasks_client.end());
        hard_tasks_client.push_back(random_number);
    }
    std::sort(hard_tasks_client.begin(), hard_tasks_client.end());


    // Show selected hard tasts for client
    qDebug() << QString("Selected hard tasks client");
    //Print selected areas
    for(uint h_task : hard_tasks_client)
    {
        qDebug() << h_task;
    }


    initialiseEnvironment(activated_areas, hard_tasks, hard_tasks_client);

    // preparint initialise ("I") server message
    QVector<int> server_task (activated_areas.size(), 0);
    QVector<int> client_task (activated_areas.size(), 0);

    initialise_buffer = "I";

    for(int i=0; i<activated_areas.size(); i++)
    {
        int char_id = 97+activated_areas[i];    // 97 is a in ASCII table
        initialise_buffer.append(QChar(char_id));

        if(std::find(hard_tasks.begin(),hard_tasks.end(), activated_areas[i]) != hard_tasks.end())
            server_task[i] = 1;

        if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), activated_areas[i]) != hard_tasks_client.end())
            client_task[i] = 1;

    }

    qDebug() << "server " << server_task;
    qDebug() << "client " << client_task;

    for(uint s_task : server_task)
    {
        initialise_buffer.append(QString::number(s_task));
    }
    for(uint c_task : client_task)
    {
        initialise_buffer.append(QString::number(c_task));
    }




}

// Only update if environment is dynamic:
void mykilobotenvironment::update() {
    //eventualmente sarà qui che gestirai il completamento delle aree
//    send_buffer = "A";
//    for(Area* a : areas)
//    {
//        send_buffer.append(QString::number(1));
//    }
}



// generate virtual sensors reading and send it to the kbs (same as for ARGOS)
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {
    //    qDebug() << QString("In updateVirtualSensor");
    // update local arrays
    kilobot_id k_id = kilobot_entity.getID();
    this->kilobots_positions[k_id] = SCALING * kilobot_entity.getPosition();
    uint8_t task_type = SOFT_TASK;

    // qDebug() << QString("saving colours");
    // update kilobot led colour (indicates the internal state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();
    if(kb_colour == lightColour::RED){
        this->kilobots_colours[k_id] = Qt::red;     // kilobot in WAITING
        qDebug() << "ReeEEEEEEEEEEEEEEEEEEEEE " << k_id;
    }
    else if(kb_colour == lightColour::BLUE){
        this->kilobots_colours[k_id] = Qt::blue;    // kilobot in LEAVING
        qDebug() << "BLUEEEEEEEEEEEEEEEEEEEEE " << k_id;
    }
    else
    {
        this->kilobots_colours[k_id] = Qt::black;   // random walking
        qDebug() << "BLack****************** " << k_id;
    }


    // ready areas from client side
    QVector<int> ready(areas.size(),0);


    if(receive_buffer.startsWith("T")){
        receive_buffer.remove(0,1); // remove the "T"
        for(int i=0; i<receive_buffer.size(); i++){
            ready[i] = QString(receive_buffer[i]).toInt();
        }
    }

    // check if inside

    bool found = false;

    // here you manage in which area is the kilobot
    for(int i=0; i<areas.size(); i++)
    {

        if(areas[i]->isCompleted(this->time, this->completed_area, ready[i]) || areas[i]->completed)
        {
            // qDebug() << "Real area";
            // areas[i]->PrintArea();

            // qDebug() << "********copied area********";
            // this->completed_area->PrintArea();

            if(this->time == this->completed_area->completed_time)
            {

                this->saveLOG = true;
            }

            if(!areas[i]->Respawn(this->time))
                continue;
        }



        // inside bigger radius (radius)
        if(areas[i]->isInside(kilobot_entity.getPosition()))
        {
            // inside small radius (radius - kilodiameter)
            if(areas[i]->isInside(kilobot_entity.getPosition(), KILO_DIAMETER) && kilobots_states[k_id] == (KilobotEnvironment::kilobot_arena_state)RANDOM_WALK)
            {
                if(std::find(areas[i]->kilobots_in_area.begin(),areas[i]->kilobots_in_area.end(), k_id) == areas[i]->kilobots_in_area.end())
                    areas[i]->kilobots_in_area.push_back(k_id);
                kilobots_states_LOG[k_id] = kilobots_states[k_id];
                kilobots_states[k_id] = (KilobotEnvironment::kilobot_arena_state)INSIDE_AREA;
                // to evaluate waiting timer you consider the area type
                task_type = areas[i]->type;
                found = true;
                break;
            }

            if( (kilobots_colours[k_id] == Qt::black || kilobots_states[k_id] == (KilobotEnvironment::kilobot_arena_state)RANDOM_WALK) ||
                ((kilobots_colours[k_id] == Qt::red || kilobots_states[k_id] == (KilobotEnvironment::kilobot_arena_state)INSIDE_AREA) && kilobots_colours[k_id] != Qt::blue)  )
            {
                if(kilobots_states[k_id] == (KilobotEnvironment::kilobot_arena_state)INSIDE_AREA && kilobots_colours[k_id] == Qt::red)
                {
                    kilobots_states_LOG[k_id] = kilobots_states[k_id];
                }

                if(std::find(areas[i]->kilobots_in_area.begin(),areas[i]->kilobots_in_area.end(), k_id) == areas[i]->kilobots_in_area.end())
                    areas[i]->kilobots_in_area.push_back(k_id);
                found = true;
                break;
            }
            else if(kilobots_colours[k_id] == Qt::blue || kilobots_states[k_id] == (KilobotEnvironment::kilobot_arena_state)LEAVING)
            {
                if(kilobots_colours[k_id] == Qt::blue)
                    qDebug() << "BLUEEEEEEEEEEEEEEEEEEEEE " << k_id;
                if(kilobots_states[k_id] == (KilobotEnvironment::kilobot_arena_state)LEAVING)
                    kilobots_states_LOG[k_id] = kilobots_states[k_id];
                kilobots_states[k_id] = (KilobotEnvironment::kilobot_arena_state)LEAVING;
                areas[i]->kilobots_in_area.erase(std::remove(areas[i]->kilobots_in_area.begin(), areas[i]->kilobots_in_area.end(), k_id),
                                          areas[i]->kilobots_in_area.end());

                found = true;
                break;
            }

        }
        // outside
        // NOTE: if kilobot passes from blue to black you should remove it from the area position
        else
        {
            areas[i]->kilobots_in_area.erase(std::remove(areas[i]->kilobots_in_area.begin(), areas[i]->kilobots_in_area.end(), k_id),
                                      areas[i]->kilobots_in_area.end());
        }

    }

    if(!found)
    {
        if(kilobots_states[k_id] == (KilobotEnvironment::kilobot_arena_state)RANDOM_WALK)
            kilobots_states_LOG[k_id] = kilobots_states[k_id];
        kilobots_states_LOG[k_id] = kilobots_states[k_id];
        kilobots_states[k_id] = (KilobotEnvironment::kilobot_arena_state)RANDOM_WALK;
    }


    qDebug() <<QString("Kilobot %1 state is: %2").arg(k_id).arg(kilobots_states[k_id]);
    qDebug() <<QString("Kilobot %1 LOG state is: %2").arg(k_id).arg(kilobots_states_LOG[k_id]);



//    qDebug() << QString("Sending message");
    // now we have everything up to date and everything we need
    // then if it is time to send the message to the kilobot send info to the kb
    if(this->time - this->lastSent[k_id] > minTimeBetweenTwoMsg && !ongoingRuntimeIdentification){
        // send if
        // black+inside = wait and complete the task
        // red+outside = task accomplished
        // blue+outside = leaving procedure from task is terminated

        /* Prepare the inividual kilobot's message         */
        /* see README.md to understand about ARK messaging */
        /* data has 3x24 bits divided as                   */
        /*   ID 10b    type 4b  data 10b     <- ARK msg    */
        /*  data[0]   data[1]   data[2]      <- kb msg     */
        /* xxxx xxxx xxyy yyzz zzzz zzzz     <- dhtf       */
        /* x bits used for kilobot id                      */
        /* y bits used for inside/outside                  */
        /* z bits used for timer to wait for others kb     */

        // RANDOM WALK ------------> INSIDE_AREA
        if( ((kilobots_states_LOG[k_id] == RANDOM_WALK && kilobots_states[k_id] == INSIDE_AREA) || (kilobots_colours[k_id] == Qt::black && kilobots_states[k_id] == INSIDE_AREA)) )
        {   
            lastSent[k_id] = this->time;
            // create and fill the message
            kilobot_message message; // this is a 24 bits field not the original kb message


            message.id = k_id;
            message.type = 1;   // sending inside to the kilobot
            if(task_type == HARD_TASK)
                message.data = 20; //seconds
            if(task_type == SOFT_TASK)
                message.data = 10; //seconds


            qDebug() << QString("Sending message to kilobot %1").arg(k_id);
            qDebug() << QString("Sending INSIDE");
            qDebug() << QString("TYPE: %1").arg(message.type);
            qDebug() << QString("Timer sent: %1").arg(message.data) << endl;

            emit transmitKiloState(message);
        }

        // INSIDE_AREA ------------> RANDOM WALK
        // LEAVING ----------------> RANDOM WALK
        else if(((kilobots_states_LOG[k_id] != RANDOM_WALK && kilobots_states[k_id] == RANDOM_WALK) || (kilobots_colours[k_id] != Qt::black && kilobots_states[k_id] == RANDOM_WALK)) )
        {
            lastSent[k_id] = this->time;
            // create and fill the message
            kilobot_message message; // this is a 24 bits field not the original kb message

            message.id = k_id;
            message.type = 0;
            message.data = 0;


            qDebug() << QString("Sending message to kilobot %1").arg(k_id);
            qDebug() << QString("Sending OUTSIDE");
            qDebug() << QString("TYPE: %1").arg(message.type);
            qDebug() << QString("Timer sent: %1").arg(message.data) << endl;

            emit transmitKiloState(message);
        }

        else
            qDebug() << QString("NOT need to send a message to kilobot %1").arg(k_id) << endl;

    }

#endif // DHTFENVIRONMENT_CPP


}




