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

    this->saveLOG = false;
    this->send_buffer = "";
    this->receive_buffer = "";
    this->initialised = false;

    // define environment:
    // call any functions to setup features in the environment (goals, homes locations and parameters).
    reset();
}


void mykilobotenvironment::initialiseAreas()
{
    double white_space = SCALING * 2 * KILO_DIAMETER;
    double radius = (((2.0*ARENA_CENTER*SCALING - white_space)/ 4.0) - white_space)/2.0;
    // QPointF areasOffset(0,1000);    //sheffield offset
    QPointF areasOffset(SHIFTX,SHIFTY);    //cnr offset

    receive_buffer.remove(0,1); // remove 1 char from position 0 (i.e. "I")
    qDebug() << "initialise buffer " << receive_buffer;
    int num_areas = receive_buffer.size() / 3;
    qDebug() << "num areas " << num_areas;
    QVector<int> activated_areas (num_areas,100);
    QString client_task = receive_buffer.right(num_areas);
    receive_buffer.chop(num_areas);
    QString server_task = receive_buffer.right(num_areas);
    receive_buffer.chop(num_areas);
    QString areas_char = receive_buffer.right(num_areas);
    receive_buffer.clear();

    for(int i=0; i<areas_char.size(); i++){
        activated_areas[i] = areas_char[i].toLatin1() - 97;
    }


    qDebug() << "areas_char" << areas_char;
    qDebug() << "areas_in" << activated_areas;
    qDebug() << "server_task" << server_task;
    qDebug() << "client_task" << client_task;
    // qDebug() << "buffer" << receive_buffer;


    for (int areaID=0; areaID<16; areaID++){
        if(std::find(activated_areas.begin(),activated_areas.end(), areaID) != activated_areas.end())
        {
            // qDebug() << "areaID" << areaID;
            QPointF areaPos((1.0+2.0*(areaID%4))*radius + (1.0+(areaID%4))*white_space, (1.0 + floor(areaID/4)*2.0 )*radius + (1.0 + floor(areaID/4))*white_space);
            areaPos += areasOffset;
            if(client_task[0] == "1")
            {
                if(server_task[0] == "1")
                    areas.push_back(new Area(areaID, HARD_TASK, HARD_TASK, areaPos, radius));
                else
                    areas.push_back(new Area(areaID, HARD_TASK, SOFT_TASK, areaPos, radius));
            }

            else
            {
                if(server_task[0] == "1")
                    areas.push_back(new Area(areaID, SOFT_TASK, HARD_TASK, areaPos, radius));
                else
                    areas.push_back(new Area(areaID, SOFT_TASK, SOFT_TASK, areaPos, radius));
            }

            client_task.remove(0,1);
            server_task.remove(0,1);
        }
    }


}
void mykilobotenvironment::reset(){
    this->time = 0;
    this->minTimeBetweenTwoMsg = 0;

    areas.clear();
    kilobots_states.clear();
    kilobots_states_LOG.clear();

    kilobots_positions.clear();
    kilobots_colours.clear();


    send_buffer = "";


}

// Only update if environment is dynamic:
void mykilobotenvironment::update() {
    //eventualmente sarà qui che gestirai il completamento delle aree
    // qDebug() << "UPDATE: " << receive_buffer;

    if(receive_buffer.startsWith("I") && this->initialised == false)
    {
        qDebug() << "*********INITIALISING ENV*********";
        this->initialised = true;
        initialiseAreas();
        receive_buffer.clear();
    }

    else if(receive_buffer.startsWith("A"))
    {
        // qDebug() << "RECEIVED:" << receive_buffer;
        receive_buffer.remove(0,1);
        QVector<int> completed(areas.size(),0);
        for(int i=0; i<receive_buffer.size(); i++){
            completed[i] = QString(receive_buffer[i]).toInt();
            if( completed[i] != (areas[i]->completed == true ? 1 : 0) )
            {
                if(completed[i] == 1)
                {
                     areas[i]->set_completed(this->time, this->completed_area);

                     qDebug() << "Kilo on area " << this->completed_area->kilobots_in_area << "time:" << this->time;
                     for(uint k : this->completed_area->kilobots_in_area)
                     {
                         kilobot_message party_message;
                         party_message.id = k;
                         party_message.type = PARTY;
                         party_message.data = 0;
                         // qDebug() << "Party for kID: " << k << "type: "<< party_message.type;
                         lastSent[k] = this->time;
                         emit transmitKiloState(party_message);
                     }

                     this->saveLOG = true;
                }
                else
                {
                    areas[i]->received_Respawn(this->time);
                }
            }
        }
        receive_buffer.clear();
    }

    // TODO : prepare the send_buffer
     send_buffer = "T";
     for(Area* a : areas)
     {
         if(a->isReady())
            send_buffer.append(QString::number(1));
         else
            send_buffer.append(QString::number(0));
     }
}



// generate virtual sensors reading and send it to the kbs (same as for ARGOS)
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {
    // qDebug() << QString("In updateVirtualSensor");
    // update local arrays
    kilobot_id k_id = kilobot_entity.getID();
    this->kilobots_positions[k_id] = kilobot_entity.getPosition();

    // qDebug() << QString("saving colours");
    // update kilobot led colour (indicates the internal state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();
    if(kb_colour == lightColour::RED){
        this->kilobots_colours[k_id] = Qt::red;     // kilobot in WAITING
        // qDebug() << "ReeEEEEEEEEEEEEEEEEEEEEE " << k_id;
    }
    else if(kb_colour == lightColour::BLUE){
        this->kilobots_colours[k_id] = Qt::blue;    // kilobot in LEAVING
        // qDebug() << "BLUEEEEEEEEEEEEEEEEEEEEE " << k_id;
    }
    else
    {
        this->kilobots_colours[k_id] = Qt::black;   // random walking
        // qDebug() << "BLack****************** " << k_id;
    }



    // check if inside

    bool found = false;
    int timer_to_send = 0;

    // here you manage in which area is the kilobot
    for(Area* a : areas)
    {

        if(a->completed)
        {
            continue;
        }



        // inside bigger radius (radius)
        if(a->isInside(kilobot_entity.getPosition()))
        {
            // inside small radius (radius - kilodiameter)
            if(a->isInside(kilobot_entity.getPosition(), KILO_DIAMETER * SCALING / 2) && kilobots_states[k_id] == RANDOM_WALK)
            {
                if(std::find(a->kilobots_in_area.begin(),a->kilobots_in_area.end(), k_id) == a->kilobots_in_area.end())
                    a->kilobots_in_area.push_back(k_id);
                kilobots_states_LOG[k_id] = kilobots_states[k_id];
                kilobots_states[k_id] = INSIDE_AREA;
                // to evaluate waiting timer you consider the area type
                timer_to_send = a->waiting_timer / 10;
                found = true;
                break;
            }

            if( (kilobots_colours[k_id] == Qt::black || kilobots_states[k_id] == RANDOM_WALK) ||
                ((kilobots_colours[k_id] == Qt::red || kilobots_states[k_id] == INSIDE_AREA) && kilobots_colours[k_id] != Qt::blue)  )
            {
                if(kilobots_states[k_id] == INSIDE_AREA && kilobots_colours[k_id] == Qt::red)
                {
                    kilobots_states_LOG[k_id] = kilobots_states[k_id];
                }

                if(std::find(a->kilobots_in_area.begin(),a->kilobots_in_area.end(), k_id) == a->kilobots_in_area.end())
                    a->kilobots_in_area.push_back(k_id);
                found = true;
                break;
            }
            else if(kilobots_colours[k_id] == Qt::blue || kilobots_states[k_id] == LEAVING)
            {
                // if(kilobots_colours[k_id] == Qt::blue)
                    // qDebug() << "BLUEEEEEEEEEEEEEEEEEEEEE " << k_id;
                if(kilobots_states[k_id] == LEAVING)
                    kilobots_states_LOG[k_id] = kilobots_states[k_id];
                kilobots_states[k_id] = LEAVING;
                a->kilobots_in_area.erase(std::remove(a->kilobots_in_area.begin(), a->kilobots_in_area.end(), k_id),
                                          a->kilobots_in_area.end());

                found = true;
                break;
            }

        }
        // outside
        // NOTE: if kilobot passes from blue to black you should remove it from the area position
        else
        {
            a->kilobots_in_area.erase(std::remove(a->kilobots_in_area.begin(), a->kilobots_in_area.end(), k_id),
                                      a->kilobots_in_area.end());
        }

    }

    if(!found)
    {
        if(kilobots_states[k_id] == RANDOM_WALK)
            kilobots_states_LOG[k_id] = kilobots_states[k_id];
        kilobots_states_LOG[k_id] = kilobots_states[k_id];
        kilobots_states[k_id] = RANDOM_WALK;
    }


    // qDebug() <<QString("Kilobot %1 state is: %2").arg(k_id).arg(kilobots_states[k_id]);
    // qDebug() <<QString("Kilobot %1 LOG state is: %2").arg(k_id).arg(kilobots_states_LOG[k_id]);



    // qDebug() << QString("Sending message");
    // now we have everything up to date and everything we need
    // then if it is time to send the message to the kilobot send info to the kb
    if(this->time - this->lastSent[k_id] > minTimeBetweenTwoMsg){
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

        kilobot_message message; // this is a 24 bits field not the original kb message
        // make sure to start clean
        message.id = 0;
        message.type = 0;
        message.data = 0;

        /***********************WALL AVOIDANCE STUFF***********************/
        // store kb rotation toward the center if the kb is too close to the border
        // this is used to avoid that the kb gets stuck in the wall
        uint8_t turning_in_msg = 0;  // 0 no turn, 1 pi/2, 2 pi, 3 3pi/2

        QPoint center ((ARENA_CENTER*SCALING)+SHIFTX, (ARENA_CENTER*SCALING)+SHIFTY);
        int distance_from_centre_x = this->kilobots_positions[k_id].x()-center.x();
        int distance_from_centre_y = this->kilobots_positions[k_id].y()-center.y();


        // Check kPos to perform wall avoidance
        if( abs(distance_from_centre_x) > (ARENA_SIZE*SCALING/2) - (2*KILO_DIAMETER) ||
            abs(distance_from_centre_y) > (ARENA_SIZE*SCALING/2) - (2*KILO_DIAMETER) ){
            // qDebug() << " COLLISION for kilobot " << k_id << " in position "<< kilobots_positions[k_id].x() << " " << kilobots_positions[k_id].y()
            //                                                             << " orientation " << qAtan2(QVector2D(kilobot_entity.getVelocity()).y(), QVector2D(kilobot_entity.getVelocity()).x());
            // get position translated w.r.t. center of arena
            QVector2D pos = QVector2D(this->kilobots_positions[k_id]);
            pos.setX(center.x() - pos.x());
            pos.setY(center.y() - pos.y());
            // get orientation (from velocity)
            QVector2D ori = QVector2D(kilobot_entity.getVelocity());
            ori.setX(ori.x()*10);
            ori.setY(ori.y()*10);
            // use atan2 to get angle between two vectors
            double angle = qAtan2(ori.y(), ori.x()) - qAtan2(pos.y(), pos.x());

            if(angle > M_PI*3/4 || angle < -M_PI*3/4) {
                 turning_in_msg = 2;
            } else if(angle < -M_PI/2) {
                turning_in_msg = 3;
            } else if(angle > M_PI/2){
                turning_in_msg = 1;
            }


            message.data = turning_in_msg;
            message.data = message.data << 4;
            // qDebug() << "ARK COLLISION MESSAGE to " << k_id << "type " << message.type << "payload " << message.data << "time:"<<this->time;
        }

        if(kilobots_states[k_id] == INSIDE_AREA && kilobots_colours[k_id] != Qt::red)
        {
            message.id = k_id;
            message.type = 1;   // sending inside to the kilobot
            message.data = message.data | timer_to_send; //seconds

            qDebug() << "ARK EXP MESSAGE to " << k_id << " INSIDE, type " << message.type << "time:"<<this->time;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }

        else if(kilobots_states[k_id] == RANDOM_WALK)
        {
            message.id = k_id;
            message.type = 0;

            qDebug() << "ARK EXP MESSAGE to " << k_id << " OUTSIDE, type " << message.type << "time:"<<this->time;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }
//        else
//            qDebug() << QString("NOT need to send a message to kilobot %1").arg(k_id) << endl;


    }

#endif // DHTFENVIRONMENT_CPP


}




