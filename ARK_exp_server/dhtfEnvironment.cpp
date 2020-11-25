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

    this->initialised_client = false;
    this->initialise_buffer = "";
    this->send_buffer = "";
    this->receive_buffer= "";
    // define environment:
    // call any functions to setup features in the environment (goals, homes locations and parameters).
    reset();
}

void mykilobotenvironment::initialiseEnvironment(QVector<int> activated_areas, QVector<uint> hard_tasks, QVector<uint> hard_tasks_client){

    double white_space = SCALING * 2 * KILO_DIAMETER;
    double radius = (((2.0*ARENA_CENTER*SCALING - white_space)/ 4.0) - white_space)/2.0;
    // qDebug() << QString("radius") << radius;
    QPointF areasOffset(SHIFTX,SHIFTY);

    for (int areaID=0; areaID<16; ++areaID){
        if(std::find(activated_areas.begin(),activated_areas.end(), areaID) != activated_areas.end())
        {
            QPointF areaPos((1.0+2.0*(areaID%4))*radius + (1.0+(areaID%4))*white_space, (1.0 + floor(areaID/4)*2.0 )*radius + (1.0 + floor(areaID/4))*white_space);
            areaPos += areasOffset;
            if(std::find(hard_tasks.begin(),hard_tasks.end(), areaID) != hard_tasks.end())
            {
                if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), areaID) != hard_tasks_client.end())
                    areas.push_back(new Area(areaID, HARD_TASK, HARD_TASK, areaPos, radius));
                else
                    areas.push_back(new Area(areaID, HARD_TASK, SOFT_TASK, areaPos, radius));
            }

            else
            {
                if(std::find(hard_tasks_client.begin(),hard_tasks_client.end(), areaID) != hard_tasks_client.end())
                    areas.push_back(new Area(areaID, SOFT_TASK, HARD_TASK, areaPos, radius));
                else
                    areas.push_back(new Area(areaID, SOFT_TASK, SOFT_TASK, areaPos, radius));
            }

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

    // Print selected areas
    QDebug dbg(QtDebugMsg); // plotting on one line even with the loop
    dbg << "Selected areas: ";
    for(int act_ar : activated_areas) {
        dbg << act_ar << ", ";
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
    send_buffer = "A";
    for(int i = 0; i < areas.size(); i++)
    {
        if(areas[i]->completed)
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
    int timer_to_send = 0;

    // here you manage in which area is the kilobot
    for(int i=0; i<areas.size(); i++)
    {

        if(areas[i]->isCompleted(this->time, this->completed_area, ready[i]) || areas[i]->completed)
        {
            // qDebug() << "Real area";
            // areas[i]->PrintArea();

            // qDebug() << "********copied area********";
            // this->completed_area->PrintArea();

            if(std::fabs(this->time - this->completed_area->completed_time) < 0.000001)
            {
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

            if(!areas[i]->Respawn(this->time))
                continue;
        }



        // inside bigger radius (radius)
        if(areas[i]->isInside(kilobot_entity.getPosition()))
        {
            // inside small radius (radius - kilodiameter)
            if(areas[i]->isInside(kilobot_entity.getPosition(), KILO_DIAMETER * SCALING / 2) && kilobots_states[k_id] == RANDOM_WALK)
            {
                if(std::find(areas[i]->kilobots_in_area.begin(),areas[i]->kilobots_in_area.end(), k_id) == areas[i]->kilobots_in_area.end())
                    areas[i]->kilobots_in_area.push_back(k_id);
                kilobots_states_LOG[k_id] = kilobots_states[k_id];
                kilobots_states[k_id] = INSIDE_AREA;

                timer_to_send = areas[i]->waiting_timer / 10;
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


                if(std::find(areas[i]->kilobots_in_area.begin(),areas[i]->kilobots_in_area.end(), k_id) == areas[i]->kilobots_in_area.end())
                    areas[i]->kilobots_in_area.push_back(k_id);
                found = true;
                break;
            }
            else if(kilobots_colours[k_id] == Qt::blue || kilobots_states[k_id] == LEAVING)
            {
//                if(kilobots_colours[k_id] == Qt::blue)
//                    qDebug() << "BLUEEEEEEEEEEEEEEEEEEEEE " << k_id;
                if(kilobots_states[k_id] == LEAVING)
                    kilobots_states_LOG[k_id] = kilobots_states[k_id];
                kilobots_states[k_id] = LEAVING;
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


        // RANDOM WALK ------------> INSIDE_AREA
        if( ((kilobots_states_LOG[k_id] == RANDOM_WALK && kilobots_states[k_id] == INSIDE_AREA) ||
             (kilobots_colours[k_id] == Qt::black && kilobots_states[k_id] == INSIDE_AREA)) )
        {
            // create and fill the message

            message.id = k_id;
            message.type = 1;   // sending inside to the kilobot
            message.data = timer_to_send; //seconds


//            qDebug() << QString("Sending message to kilobot %1").arg(k_id);
//            qDebug() << QString("Sending INSIDE");
//            qDebug() << QString("TYPE: %1").arg(message.type);
//            qDebug() << QString("Timer sent: %1").arg(message.data) << endl;

            // qDebug() << "ARK EXP MESSAGE to " << k_id << " INSIDE, type " << message.type << "time:"<<this->time;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }

        // INSIDE_AREA ------------> RANDOM WALK
        // LEAVING ----------------> RANDOM WALK
        else if((kilobots_states_LOG[k_id] != RANDOM_WALK && kilobots_states[k_id] == RANDOM_WALK) /*|| (kilobots_colours[k_id] != Qt::black && kilobots_states[k_id] == RANDOM_WALK))*/ )
        {

            message.id = k_id;
            message.type = 0;
            message.data = 0;

//            qDebug() << QString("Sending message to kilobot %1").arg(k_id);
//            qDebug() << QString("Sending OUTSIDE");
//            qDebug() << QString("TYPE: %1").arg(message.type);
//            qDebug() << QString("Timer sent: %1").arg(message.data) << endl;

            // qDebug() << "ARK EXP MESSAGE to " << k_id << " OUTSIDE, type " << message.type << "time:"<<this->time;
            lastSent[k_id] = this->time;
            emit transmitKiloState(message);
        }

        // Check kPos to perform wall avoidance
        else if(abs(distance_from_centre_x) > (ARENA_SIZE*SCALING/2) - (2*KILO_DIAMETER) ||
                abs(distance_from_centre_y) > (ARENA_SIZE*SCALING/2) - (2*KILO_DIAMETER)) {
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


            if(turning_in_msg != 0)
            {
                // store angle (no need if 0)
                message.id = k_id;
                message.type = 2;   // sending colliding to the kilobot
                message.data = turning_in_msg;
                // qDebug() << "ARK COLLISION MESSAGE to " << k_id << "type " << message.type << "payload " << message.data << "time:"<<this->time;
                lastSent[k_id] = this->time;
                emit transmitKiloState(message);
            }
        }
//        else
//            qDebug() << QString("NOT need to send a message to kilobot %1").arg(k_id) << endl;


    }

#endif // DHTFENVIRONMENT_CPP


}




