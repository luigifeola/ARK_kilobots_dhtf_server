#ifndef DHTFENVIRONMENT_CPP
#define DHTFENVIRONMENT_CPP

#include "dhtfEnvironment.h"
#include "area.h"

#include "kilobot.h"

#include <math.h>
#include <stdlib.h>
#include <random>
#include <functional>

#include <QVector>
#include <QVector2D>
#include <QLineF>
#include <QDebug>
#include <QtMath>
#include <QColor>

namespace {
    const double reachable_distance = (ARENA_SIZE/2) - 2.0*KILO_DIAMETER;
    const int num_sectors = 8;
    const QVector2D left_direction (1.0,0.0);


    const double kTask_radius = 6.0*CM_TO_PIXEL;
    const int kHard_task_requirement = 4;
    const int kSoft_task_requirement = 2;

    const double kRespawnTimer = 60;
    const double kTimerMultiplier = 30;
}

double mykilobotenvironment::normAngle(double angle){
    while (angle > 180) angle = angle - 360;
    while (angle < -180) angle = angle + 360;
    return angle;
}

QVector2D mykilobotenvironment::VectorRotation2D (double angle, QVector2D vec){
//     qDebug() << "ENV.2D Rotation";
    QVector2D rotated_vector;
    double kx = (cos(angle)* vec.x()) + (-1.0*sin(angle) * vec.y());
    double ky = (sin(angle) * vec.x()) + (cos(angle) * vec.y());
    rotated_vector.setX(kx);
    rotated_vector.setY(ky);
    return rotated_vector;
}

QVector<int> mykilobotenvironment::proximity_sensor(QVector2D obstacle_direction, double kilo_rotation, int num_bit){
//    qDebug() << "ENV. proximity_sensor";
    double sector = M_PI_2 / (num_bit/2.0);
    QVector<int> proximity;

    // qDebug() << "kilo_ori" << qRadiansToDegrees(kilo_rotation);
    for(int i=0; i<num_bit; i++)
    {
        QVector2D sector_dir_start = VectorRotation2D((kilo_rotation+M_PI_2 - i * sector), left_direction);
        QVector2D sector_dir_end = VectorRotation2D((kilo_rotation+M_PI_2 - (i+1) * sector), left_direction);

//         qDebug() << "wall-dir" << obstacle_direction;
//         qDebug() << "a-dir" << sector_dir_start;
//         qDebug() << "b-dir" << sector_dir_end;

        if( QVector2D::dotProduct(obstacle_direction, sector_dir_start) >=0 ||
            QVector2D::dotProduct(obstacle_direction, sector_dir_end) >=0    )
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

//    qDebug() << "ENV. mykilobotenvironment";
    this->saveLOG = false;
    // call any functions to setup features in the environment (goals, homes locations and parameters).
    reset();
}


bool mykilobotenvironment::DistantEnoughTasks(QPointF some_position)
{
//    qDebug() << "ENV. DistantEnoughTasks";
    for(size_t i = 0; i<areas.size(); i++)
    {
        double distance = pow((areas[i]->position-some_position).x(),2) + pow((areas[i]->position-some_position).y(),2);
        if (distance < pow(2.0*areas[i]->radius + 2.0*CM_TO_PIXEL,2))
            return false;
    }

    return true;
}


void mykilobotenvironment::reset()
{
//    qDebug() << "ENV. reset";
    this->time = 0;
    this->minTimeBetweenTwoMsg = 0;

    areas.clear();
    kilobots_states.clear();

    kilobots_positions.clear();
    kilobots_colours.clear();
    kilobots_in_collision.clear();

    std::default_random_engine re;
    re.seed(200);
//    re.seed(qrand());

    /**TODO: fix task virtual set-up*/
    for (int i = 0; i < ACTIVE_AREAS / 2; i++)
    {
        int x_pos, y_pos, y_opposite;
        QVector2D shifted_pos;
        do
        {

            QPoint k_center ((ARENA_CENTER+SHIFTX), (ARENA_CENTER+SHIFTY));

//            std::uniform_int_distribution<int> distr_x(7.0*CM_TO_PIXEL, ARENA_SIZE/2 - 10*CM_TO_PIXEL);
            std::uniform_int_distribution<int> distr_x(10.0*CM_TO_PIXEL, k_center.x() - kTask_radius - 0.5 * CM_TO_PIXEL);
            x_pos = distr_x(re);

//            std::uniform_int_distribution<int> distr_y(-1.0 * (95.0*CM_TO_PIXEL / 2.0 - 10*CM_TO_PIXEL), +1.0 * (95.0*CM_TO_PIXEL / 2.0 - 10*CM_TO_PIXEL));
            std::uniform_int_distribution<int> distr_y(10.0*CM_TO_PIXEL, 2.0*k_center.y()-10.0*CM_TO_PIXEL);
            y_pos = distr_y(re);

            // get position translated w.r.t. center of arena
            QVector2D k_pos = QVector2D(x_pos, y_pos);
            shifted_pos = QVector2D((k_pos.x() - k_center.x()), -1.0*(k_pos.y() - k_center.y()) );

            y_opposite = (shifted_pos.y()>0)? 1 : -1;

        } while (pow(shifted_pos.x(),2)+pow(shifted_pos.y(),2) > (pow(ARENA_SIZE/2 - 10.0*CM_TO_PIXEL,2)) || !DistantEnoughTasks(QPointF(x_pos, y_pos)));


        QPointF oppposite_pos = QPointF(x_pos+2.0*abs(shifted_pos.x())+0.5*CM_TO_PIXEL, y_pos+y_opposite*2.0*abs(shifted_pos.y()));

        std::uniform_int_distribution<int> distr_true_false(0,1);
        int b = distr_true_false(re);
        if(b)
        {
            areas.push_back(new Area(i, HARD_TASK, QPointF(x_pos, y_pos), kTask_radius, kHard_task_requirement, kRespawnTimer, kTimerMultiplier));
            areas.push_back(new Area(i + ACTIVE_AREAS / 2, SOFT_TASK, oppposite_pos, kTask_radius, kSoft_task_requirement, kRespawnTimer, kTimerMultiplier));
        }
        else
        {
            areas.push_back(new Area(i, SOFT_TASK, QPointF(x_pos, y_pos), kTask_radius, kSoft_task_requirement, kRespawnTimer, kTimerMultiplier));
            areas.push_back(new Area(i + ACTIVE_AREAS / 2, HARD_TASK, oppposite_pos, kTask_radius, kHard_task_requirement, kRespawnTimer, kTimerMultiplier));
        }
    }

}

// Only update if environment is dynamic:
void mykilobotenvironment::update() {
//    qDebug() << "ENV. update";
}



// generate virtual sensors reading and send it to the kbs (same as for ARGOS)
void mykilobotenvironment::updateVirtualSensor(Kilobot kilobot_entity) {   
//     qDebug() << QString("ENV. In updateVirtualSensor");
    // update local arrays
    kilobot_id k_id = kilobot_entity.getID();
    this->kilobots_positions[k_id] = kilobot_entity.getPosition();

    // update kilobot led colour (indicates the internal state of the kb)
    lightColour kb_colour = kilobot_entity.getLedColour();

    if(qRound(this->time*10.0f) <= 1.0f*10.0f )
        this->kilobots_colours[k_id] = Qt::black;

    else if(kb_colour == lightColour::RED){
        this->kilobots_colours[k_id] = Qt::red;     // kilobot in LEAVING
//        qDebug()<< "time:"<<this->time << k_id << " detected red";
    }
    else if(kb_colour == lightColour::BLUE){
        this->kilobots_colours[k_id] = Qt::blue;    // kilobot in WAITING
//        qDebug()<< "time:"<<this->time << k_id << " detected blue";
    }
    else
    {
        this->kilobots_colours[k_id] = Qt::black;   // random walking
    }



    // check if inside

    bool found = false;
    int timer_to_send = 0;

    // here you manage in which area is the kilobot
    for(int i=0; i<areas.size(); i++)
    {

        if( areas[i]->isCompleted(this->time, this->completed_area) || (areas[i]->completed) )
        {
            // qDebug() << "Real area";
            // areas[i]->PrintArea();

            // qDebug() << "********copied area********";
            // this->completed_area->PrintArea();


            /**** PARTY MESSAGE *****/
//            if(std::fabs(this->time - this->completed_area->completed_time) < 0.2f*10.f)
            if(std::fabs(this->time - this->completed_area->completed_time) < 0.0000001)
            {
//                 qDebug() << "Kilo on area " << this->completed_area->kilobots_in_area << "time:" << this->time;
//                for(uint k : this->completed_area->kilobots_in_area)
//                {
//                    kilobot_message party_message;
//                    party_message.id = k;
//                    party_message.type = PARTY;
//                    party_message.data = 0;
//                    qDebug() << "time:"<<this->time << " PARTY MESSAGE to kIDs" << k;
//                    lastSent[k] = this->time;
//                    emit transmitKiloState(party_message);
//                }

                this->saveLOG = true;
            }

            if(!areas[i]->Respawn(this->time))
                continue;
        }



        // inside bigger radius (radius)
        if(areas[i]->isInside(kilobot_entity.getPosition()))
        {

            if(kilobots_colours[k_id] == Qt::red)
            {
                kilobots_states[k_id] = LEAVING;
                areas[i]->kilobots_in_area.erase(std::remove(areas[i]->kilobots_in_area.begin(), areas[i]->kilobots_in_area.end(), k_id),
                                          areas[i]->kilobots_in_area.end());

                found = true;
                break;
            }

            else
            {
                if(std::find(areas[i]->kilobots_in_area.begin(),areas[i]->kilobots_in_area.end(), k_id) == areas[i]->kilobots_in_area.end())
                    areas[i]->kilobots_in_area.push_back(k_id);

                // inside small radius (radius - kilodiameter)
                if(areas[i]->isInside(kilobot_entity.getPosition(), KILO_DIAMETER/2.0))
                {
                    kilobots_states[k_id] = INSIDE_AREA;

                    timer_to_send = areas[i]->waiting_timer / 10;
                }

                found = true;
                break;
            }

        }
        // outside
        // NOTE: if kilobot passes from blue to black you should remove it from the area position
        //if not in this area remove k_id from it
        else
        {
            areas[i]->kilobots_in_area.erase(std::remove(areas[i]->kilobots_in_area.begin(), areas[i]->kilobots_in_area.end(), k_id),
                                      areas[i]->kilobots_in_area.end());
        }

    }

    // if in no area
    if(!found)
    {
        kilobots_states[k_id] = RANDOM_WALK;
    }


    // qDebug() <<QString("Kilobot %1 state is: %2").arg(k_id).arg(kilobots_states[k_id]);


    // now we have everything up to date and everything we need
    // then if it is time to send the message to the kilobot send info to the kb
    if(this->time - this->lastSent[k_id] > minTimeBetweenTwoMsg)
    {
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

        QPoint k_center ((ARENA_CENTER+SHIFTX), (ARENA_CENTER+SHIFTY));
        // get position translated w.r.t. center of arena
        QVector2D k_pos = QVector2D(this->kilobots_positions[k_id]);

//        bool colliding_backup = kilobots_in_collision[k_id];
        kilobots_in_collision[k_id] = pow(k_pos.x()-k_center.x(),2)+pow(k_pos.y()-k_center.y(),2) > (pow(reachable_distance,2));

        /* get orientation (from velocity) in radians in [-3.14,3.14]*/
        QVector2D k_ori = QVector2D(kilobot_entity.getVelocity());
        k_ori.setX(k_ori.x()*10);
        k_ori.setY(k_ori.y()*10);
        // qDebug() << "Orientation: " << normAngle( qRadiansToDegrees(qAtan2(-k_ori.y(), k_ori.x())) );

        double k_rotation = qAtan2(-k_ori.y(), k_ori.x());
//        qDebug() << "orientation: " << normAngle( qRadiansToDegrees(k_rotation) );  //angolo in [-180, 180] gradi


        if( (kilobots_states[k_id] == INSIDE_AREA) /*&& kilobots_colours[k_id] != Qt::blue*/)
        {
            message.id = k_id;
            message.type = INSIDE_AREA;   // sending inside to the kilobot
            message.data = timer_to_send; //seconds

//            qDebug() << "time:"<<this->time << " ARK EXP MESSAGE to " << k_id << " INSIDE, type " << message.type;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }

        else if(kilobots_in_collision[k_id])
        {
            double collision_angle = qAtan2(-1.0*(k_pos.y()-k_center.y()) , k_pos.x()-k_center.x());
            QVector2D collision_direction = QVector2D(reachable_distance*qCos(collision_angle+M_PI),reachable_distance*qSin(collision_angle+M_PI)).normalized();

//            qDebug() << "collision angle: " << qRadiansToDegrees(collision_angle) ;
//            qDebug() << "normalized" << collision_direction.normalized();
//            qDebug() << "direction rotated" << collision_direction;
            QVector<int> proximity = proximity_sensor(collision_direction, k_rotation, num_sectors);
            proximity_decimal = std::accumulate(proximity.begin(), proximity.end(), 0, [](int x, int y) { return (x << 1) + y; });

//            if(!colliding_backup && colliding_backup != kilobots_in_collision[k_id] && proximity_decimal!=0)
//            {
//                qDebug() << k_id << " COLLIDING! -> " << proximity;
//            }

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

        else if(kilobots_states[k_id] == RANDOM_WALK)
        {
            message.id = k_id;
            message.type = RANDOM_WALK;   // sending OUTSIDE
            message.data = 0;

//            qDebug() << "time:"<<this->time << " ARK EXP MESSAGE to " << k_id << " RANDOM_WALK, type " << message.type;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }
//        else
//            qDebug() << QString("NOT need to send a message to kilobot %1").arg(k_id) << endl;


    }

#endif // DHTFENVIRONMENT_CPP


}




