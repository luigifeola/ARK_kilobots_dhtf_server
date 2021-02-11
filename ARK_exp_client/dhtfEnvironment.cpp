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

namespace {
    const QVector2D up_direction (0.0,-1.0);
    const QVector2D down_direction (0.0,1.0);
    const QVector2D left_direction (1.0,0.0);
    const QVector2D right_direction (-1.0,0.0);
    const double reachable_distance = (ARENA_SIZE*SCALING/2) - (2*KILO_DIAMETER);
    const int num_sectors = 8;
}

double mykilobotenvironment::normAngle(double angle){
    while (angle > 180) angle = angle - 360;
    while (angle < -180) angle = angle + 360;
    return angle;
}

QVector2D mykilobotenvironment::VectorRotation2D (double angle, QVector2D vec){
    // qDebug() << "2D Rotation";
    QVector2D rotated_vector;
    double kx = (cos(angle)* vec.x()) + (-1.0*sin(angle) * vec.y());
    double ky = (sin(angle) * vec.x()) + (cos(angle) * vec.y());
    rotated_vector.setX(kx);
    rotated_vector.setY(ky);
    return rotated_vector;
}

QVector<int> mykilobotenvironment::proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit){
    double sector = M_PI_2 / (num_bit/2.0);
    QVector<int> proximity;
    // qDebug() << "kilo_ori" << qRadiansToDegrees(kilo_rotation);
    for(int i=0; i<num_bit; i++)
    {
        QVector2D sector_dir_a = VectorRotation2D((kilo_rotation+M_PI_2 - i * sector), left_direction);
        QVector2D sector_dir_b = VectorRotation2D((kilo_rotation+M_PI_2 - (i+1) * sector), left_direction);

        // qDebug() << "wall-dir" << obstacle_direction;
        // qDebug() << "a-dir" << sector_dir_a;
        // qDebug() << "b-dir" << sector_dir_b;

        if( QVector2D::dotProduct(obstacle_direction, sector_dir_a) >=0 ||
            QVector2D::dotProduct(obstacle_direction, sector_dir_b) >=0    )
        {
            proximity.push_back(0);
        }
        else{
            proximity.push_back(1);
        }
    }

    return proximity;
}

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
    QPointF areasOffset(SHIFTX,SHIFTY);

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
        qDebug() << "************************************";
        qDebug() << "*********INITIALISING ENV*********";
        qDebug() << "************************************";
        qDebug() << "************************************";
        qDebug() << "************************************";
        qDebug() << "************************************";
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
                         lastSent[k] = this->time;
                         qDebug() << "time:" << this->time << " ARK PARTY MESSAGE to " << k ;
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
        this->kilobots_colours[k_id] = Qt::red;     // kilobot in LEAVING
        // qDebug() << "ReeEEEEEEEEEEEEEEEEEEEEE " << k_id;
    }
    else if(kb_colour == lightColour::BLUE){
        this->kilobots_colours[k_id] = Qt::blue;    // kilobot in WAITING
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

            if(kilobots_colours[k_id] == Qt::red || kilobots_states[k_id] == LEAVING)
            {
                kilobots_states[k_id] = LEAVING;
                a->kilobots_in_area.erase(std::remove(a->kilobots_in_area.begin(), a->kilobots_in_area.end(), k_id),
                                          a->kilobots_in_area.end());

                found = true;
                break;
            }

            else
            {
                if(std::find(a->kilobots_in_area.begin(),a->kilobots_in_area.end(), k_id) == a->kilobots_in_area.end())
                    a->kilobots_in_area.push_back(k_id);

                // inside small radius (radius - kilodiameter)
                if(a->isInside(kilobot_entity.getPosition(), KILO_DIAMETER * SCALING / 2))
                {
                    kilobots_states[k_id] = INSIDE_AREA;

                    timer_to_send = a->waiting_timer / 10;
                }

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

        /* Prepare the inividual kilobot's message                   */
        /* see README.md to understand about ARK messaging           */
        /* data has 3x24 bits divided as                             */
        /*   ID 10b    type 4b  data 10b     <- ARK msg              */
        /*  data[0]   data[1]   data[2]      <- kb msg               */
        /* xxxx xxxx xxyy yy// wwww zzzz     <- dhtf                 */
        /* x bits used for kilobot id                                */
        /* y bits used for inside/outside                            */
        /* w bits used for wall avoidance                            */
        /* z bits used for timer to wait for others kb (if inside)   */

        kilobot_message message; // this is a 24 bits field not the original kb message
        // make sure to start clean
        message.id = 0;
        message.type = 0;
        message.data = 0;

        /***********************WALL AVOIDANCE STUFF***********************/
        // store kb rotation toward the center if the kb is too close to the border
        // this is used to avoid that the kb gets stuck in the wall
        uint8_t proximity_decimal = 0;  // 0 no turn, 1 pi/2, 2 pi, 3 3pi/2

        QPoint k_center ((ARENA_CENTER*SCALING)+SHIFTX, (ARENA_CENTER*SCALING)+SHIFTY);
        // get position translated w.r.t. center of arena
        QVector2D k_pos = QVector2D(this->kilobots_positions[k_id]);
        QVector2D shifted_pos((k_pos.x() - k_center.x()), -1.0*(k_pos.y() - k_center.y()) );
        // qDebug() << "pos:" << k_pos.x() << ' ' << k_pos.y() << "\tShifted:" << shifted_pos.x() << ' ' <<  shifted_pos.y();



        if( (kilobots_states[k_id] == INSIDE_AREA) /*&& kilobots_colours[k_id] != Qt::blue*/)
        {
            message.id = k_id;
            message.type = INSIDE_AREA;   // sending inside to the kilobot
            message.data = timer_to_send; //seconds

            // qDebug() << "time:"<<this->time << " ARK EXP MESSAGE to " << k_id << " INSIDE, type " << message.type;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }

        else if( abs(shifted_pos.x()) > reachable_distance ||
                 abs(shifted_pos.y()) > reachable_distance )
        {
            // qDebug() << " COLLISION for kilobot " << k_id << " in position "<< kilobots_positions[k_id].x() << " " << kilobots_positions[k_id].y()
            //                                                             << " orientation " << qAtan2(QVector2D(kilobot_entity.getVelocity()).y(), QVector2D(kilobot_entity.getVelocity()).x());
            // get position translated w.r.t. center of arena


            // get orientation (from velocity)
            QVector2D k_ori = QVector2D(kilobot_entity.getVelocity());
            k_ori.setX(k_ori.x()*10);
            k_ori.setY(k_ori.y()*10);
            // qDebug() << "Orientation: " << normAngle( qRadiansToDegrees(qAtan2(-k_ori.y(), k_ori.x())) );

            double k_rotation = qAtan2(-k_ori.y(), k_ori.x());
            // double angle = k_rotation - qAtan2(shifted_pos.y(), shifted_pos.x());
            // get angle wrt center position
            /*qDebug() << normAngle( qRadiansToDegrees(qAtan2(-k_ori.y(), k_ori.x())) )
                     << normAngle( qRadiansToDegrees(qAtan2(shifted_pos.y(), shifted_pos.x())) ) ;
                     << normAngle( qRadiansToDegrees(angle) );*/

            // qDebug() << "Pos:" << k_pos << "\tRotation" << k_rotation;
            QVector<int> proximity;
            // TODO : Understand here what you need
            if(shifted_pos.x() > reachable_distance){
                // qDebug()<< "RIGHT";
                proximity = proximity_sensor(right_direction, k_rotation, num_sectors);
            }
            else if(shifted_pos.x() < -1.0*reachable_distance){
                // qDebug()<< "LEFT";
                proximity = proximity_sensor(left_direction, k_rotation, num_sectors);
            }

            if(shifted_pos.y() > reachable_distance){
                // qDebug()<< "UP";
                if(proximity.isEmpty())
                    proximity = proximity_sensor(up_direction, k_rotation, num_sectors);
                else{
                    QVector<int> prox = proximity_sensor(up_direction, k_rotation, num_sectors);
                    QVector<int> elementwiseOr;
                    elementwiseOr.reserve(prox.size());
                    std::transform( proximity.begin(), proximity.end(), prox.begin(),
                            std::back_inserter( elementwiseOr ), std::logical_or<>() );

                    proximity = elementwiseOr;
                }
            }
            else if(shifted_pos.y() < -1.0*reachable_distance){
                // qDebug()<< "DOWN";
                if(proximity.isEmpty())
                    proximity = proximity_sensor(down_direction, k_rotation, num_sectors);
                else{
                    QVector<int> prox = proximity_sensor(down_direction, k_rotation, num_sectors);
                    QVector<int> elementwiseOr;
                    elementwiseOr.reserve(prox.size());
                    std::transform( proximity.begin(), proximity.end(), prox.begin(),
                            std::back_inserter( elementwiseOr ), std::logical_or<>() );

                    proximity = elementwiseOr;
                }
            }

            proximity_decimal = std::accumulate(proximity.begin(), proximity.end(), 0, [](int x, int y) { return (x << 1) + y; });
            // qDebug() <<proximity << "\tDec: " << proximity_decimal;


            if( proximity_decimal!= 0 )
            {
                message.id = k_id;
                message.type = kilobots_states[k_id];
                message.data = proximity_decimal;

                // qDebug() << "time:"<<this->time<< " ARK COLLISION MESSAGE to " << k_id << "type " << message.type << "payload " << message.data;
                lastSent[k_id] = this->time;
                emit transmitKiloState(message);
            }

        }

        else if(kilobots_states[k_id] == RANDOM_WALK &&
               (kilobots_colours[k_id] == Qt::blue || kilobots_colours[k_id] == Qt::red))
        {
            message.id = k_id;
            message.type = RANDOM_WALK;   // sending OUTSIDE
            message.data = 0;

            // qDebug() << "time:"<<this->time << " ARK EXP MESSAGE to " << k_id << " INSIDE, type " << message.type;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }
//        else
//            qDebug() << QString("NOT need to send a message to kilobot %1").arg(k_id) << endl;


    }

#endif // DHTFENVIRONMENT_CPP


}




