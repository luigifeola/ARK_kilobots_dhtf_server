#include "kilobot_ALF_dhtf.h"

namespace
{
    const int port = 7001;

    // environment setup
    const double kEpsilon = 0.0001;
    // const double kScaling = 1.0 / 4.0;      //for no scaling set kScaling=0.5
    const double kKiloDiameter = 0.033;
    const double kScaling = 1.0;
    double vArena_size = 0.25;
    double vDistance_threshold = vArena_size / 2.0 - 2.0 * kKiloDiameter;
    // TODO :  forse kTask_radius non ti serve
    const double kTask_radius = 0.06;
    // avoid to choose corner areas
    const std::vector<int> vForbidden({0, 3, 12, 15});

    // wall avoidance stuff
    const CVector2 up_direction(0.0, -1.0);
    const CVector2 down_direction(0.0, 1.0);
    const CVector2 left_direction(1.0, 0.0);
    const CVector2 right_direction(-1.0, 0.0);
    const int kProximity_bits = 8;
    int internal_counter = 0;

    // fake client message
    const std::string fake_client_message = "T11111111";
}

CALFClientServer::CALFClientServer() : m_unDataAcquisitionFrequency(10)
{
    c_rng = CRandom::CreateRNG("argos");
}

CALFClientServer::~CALFClientServer()
{
}

void CALFClientServer::Reset()
{
    m_kiloOutput.close();
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_areaOutput.close();
    m_areaOutput.open(m_strAreaOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_taskOutput.close();
    m_taskOutput.open(m_strTaskOutputFileName, std::ios_base::trunc | std::ios_base::out);
}

void CALFClientServer::Destroy()
{
    m_kiloOutput.close();
    m_areaOutput.close();
    m_taskOutput.close();

    if (mode == "SERVER")
    {
        close(clientSocket);
    }
    if (mode == "CLIENT")
    {
        close(serverSocket);
    }
}

void CALFClientServer::Init(TConfigurationNode &t_node)
{
    CALF::Init(t_node);

    /*********** LOG FILES *********/
    m_kiloOutput.open(m_strKiloOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_areaOutput.open(m_strAreaOutputFileName, std::ios_base::trunc | std::ios_base::out);
    m_taskOutput.open(m_strTaskOutputFileName, std::ios_base::trunc | std::ios_base::out);

    /* Read parameters from .argos*/
    TConfigurationNode &tModeNode = GetNode(t_node, "extra_parameters");
    GetNodeAttribute(tModeNode, "mode", mode);
    GetNodeAttribute(tModeNode, "ip_addr", IP_ADDR);
    GetNodeAttribute(tModeNode, "augmented_knowledge", augmented_knowledge);
    GetNodeAttribute(tModeNode, "soft_requirement", vSoftRequiredKilobots);
    GetNodeAttribute(tModeNode, "hard_requirement", vHardRequiredKilobots);

    /* Read arena parameters */
    vArena_size = argos::CSimulator::GetInstance().GetSpace().GetArenaSize().GetX();
    vDistance_threshold = vArena_size / 2.0 - 0.04;
    // LOG print
    // std::cout << "Arena size: " << vArena_size << "\n";

    /* Randomly select the desired number of tasks between the available ones, set color and communicate them to the client */
    if (mode == "SERVER")
    {
        random_seed = GetSimulator().GetRandomSeed();
        // GetNodeAttribute(tModeNode, "random_seed", random_seed);
        GetNodeAttribute(tModeNode, "desired_num_of_areas", desired_num_of_areas);
        GetNodeAttribute(tModeNode, "hard_tasks", hard_tasks);
        GetNodeAttributeOrDefault(tModeNode, "mixed", mixed, false);

        Initialise_environment();

        std::cout << "***********Active areas*****************\n";
        for (int ac_ar : activated_areas)
        {
            std::cout << ac_ar << '\t';
        }
        std::cout << std::endl;

        std::cout << "Server hard tasks id\n";
        for (int h_t : hard_tasks_vec)
        {
            std::cout << h_t << '\t';
        }
        std::cout << std::endl;

        if (mixed == false)
        {
            std::cout << "Client hard tasks id\n";
            for (int h_t_c : hard_tasks_client_vec)
            {
                std::cout << h_t_c << '\t';
            }
            std::cout << std::endl;
        }
        else
            std::cout << "Client hard tasks is dual of server\n";
        /* 0-1 vector indicatind if the active area is hard or soft type */
        // preparint initialise ("I") server message
        std::vector<int> server_task_type(activated_areas.size(), 0);
        std::vector<int> client_task_type(activated_areas.size(), 0);

        initialise_buffer = "I";

        for (int i = 0; i < activated_areas.size(); i++)
        {
            int char_id = 97 + activated_areas[i]; // 97 is a in ASCII table
            char A = static_cast<char>(char_id);
            std::string s(1, A);
            initialise_buffer.append(s);

            if (mixed == false)
            {
                if (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), activated_areas[i]) != hard_tasks_vec.end())
                    server_task_type[i] = 1;

                if (std::find(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end(), activated_areas[i]) != hard_tasks_client_vec.end())
                    client_task_type[i] = 1;
            }
            else
            {
                if (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), activated_areas[i]) != hard_tasks_vec.end())
                    server_task_type[i] = 1;
                else
                    client_task_type[i] = 1;
            }
        }

        for (int s_task : server_task_type)
        {
            initialise_buffer.append(std::to_string(s_task));
        }
        for (int c_task : client_task_type)
        {
            initialise_buffer.append(std::to_string(c_task));
        }

        // LOG print
        // std::cout << "initialise_buffer: " << initialise_buffer << std::endl;
    }

    Initialise_socket();
}

/**
 * ENVIRONMENT INITIALISATION
 */
void CALFClientServer::Initialise_environment()
{
    /* Select areas */
    srand(random_seed);
    // TODO : magari 15 lo carichi dal .argos
    const int max_area_id = 15;

    /* GENERATE RANDOM IDs AND RANDOM HARD TASK for server and client*/
    std::default_random_engine re;
    re.seed(random_seed);

    /* Active areas ID */
    while (activated_areas.size() < desired_num_of_areas)
    {
        if (desired_num_of_areas - 1 > max_area_id)
        {
            std::cerr << "Requested more areas then the available ones, WARNING!";
        }

        std::uniform_int_distribution<int> distr(0, max_area_id);
        int random_number;
        do
        {
            random_number = distr(re);
        } while ((std::find(activated_areas.begin(), activated_areas.end(), random_number) != activated_areas.end()) ||
                 (std::find(vForbidden.begin(), vForbidden.end(), random_number) != vForbidden.end()));
        activated_areas.push_back(random_number);
    }
    std::sort(activated_areas.begin(), activated_areas.end());

    /* Hard task for the server */
    while (hard_tasks_vec.size() < hard_tasks)
    {
        std::uniform_int_distribution<int> distr(0, max_area_id);
        int random_number;
        do
        {
            random_number = distr(re);
        } while (std::find(activated_areas.begin(), activated_areas.end(), random_number) == activated_areas.end() ||
                 std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), random_number) != hard_tasks_vec.end());
        hard_tasks_vec.push_back(random_number);
    }
    std::sort(hard_tasks_vec.begin(), hard_tasks_vec.end());

    /* Hard task for the client */
    /* WARNING : capire meglio l'if successivo */
    if (mixed == false)
    {
        while (hard_tasks_client_vec.size() < hard_tasks)
        {
            std::uniform_int_distribution<int> distr(0, max_area_id);
            int random_number;
            do
            {
                random_number = distr(re);
            } while (std::find(activated_areas.begin(), activated_areas.end(), random_number) == activated_areas.end() ||
                     std::find(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end(), random_number) != hard_tasks_client_vec.end());
            hard_tasks_client_vec.push_back(random_number);
        }
        std::sort(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end());
    }

    double white_space = kKiloDiameter;
    double radius = (((vArena_size - white_space) / 4.0) - white_space) / 2.0;
    // std::cout << "Radius: " << radius << "\n";
    CVector2 areasOffset(-0.25, -0.25);

    for (int areaID = 0; areaID < max_area_id + 1; ++areaID)
    {
        if (std::find(activated_areas.begin(), activated_areas.end(), areaID) != activated_areas.end())
        {
            CVector2 areaPos((1.0 + 2.0 * (areaID % 4)) * radius + (1.0 + (areaID % 4)) * white_space, (1.0 + floor(areaID / 4) * 2.0) * radius + (1.0 + floor(areaID / 4)) * white_space);
            areaPos += areasOffset;
            if (std::find(hard_tasks_vec.begin(), hard_tasks_vec.end(), areaID) != hard_tasks_vec.end())
            {
                if (std::find(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end(), areaID) != hard_tasks_client_vec.end())
                    multiArea.push_back(new Area(areaID, HARD_TASK, HARD_TASK, areaPos, radius, vHardRequiredKilobots));
                else
                    multiArea.push_back(new Area(areaID, HARD_TASK, SOFT_TASK, areaPos, radius, vHardRequiredKilobots));
            }

            else
            {
                if (std::find(hard_tasks_client_vec.begin(), hard_tasks_client_vec.end(), areaID) != hard_tasks_client_vec.end())
                    multiArea.push_back(new Area(areaID, SOFT_TASK, HARD_TASK, areaPos, radius, vSoftRequiredKilobots));
                else
                    multiArea.push_back(new Area(areaID, SOFT_TASK, SOFT_TASK, areaPos, radius, vSoftRequiredKilobots));
            }
            // LOG print
            // std::cout << "Area " << areaID << ", position: " << areaPos << std::endl;
        }
    }
}

/**
 * SOCKET INITIALISATION
 */
void CALFClientServer::Initialise_socket()
{
    /* Initializations */
    bytesReceived = -1;
    memset(storeBuffer, 0, 30); // set to 0 the 30 elements in storeBuffer
    initialised = false;

    /* Opening communication port */
    serverSocket = socket(AF_INET, SOCK_STREAM, 0);

    std::string ipAddress = IP_ADDR;
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_addr.s_addr = INADDR_ANY;
    hint.sin_port = htons(port);
    inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

    if (mode == "SERVER")
    {
        bind(serverSocket, (sockaddr *)&hint, sizeof(hint));
        listen(serverSocket, SOMAXCONN);
        sockaddr_in client;
        socklen_t clientSize = sizeof(client);
        clientSocket = accept(serverSocket, (sockaddr *)&client, &clientSize);
        char host[NI_MAXHOST];
        char service[NI_MAXSERV];
        memset(host, 0, NI_MAXHOST);
        memset(service, 0, NI_MAXSERV);
        if (getnameinfo((sockaddr *)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0)
        {
            std::cout << host << " connected on port " << service << std::endl;
            std::cout << "Somebody has connected on port " << service << std::endl;
        }
        else
        {
            inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
            std::cout << "Somebody has connected on port " << service << std::endl;
        }
        close(serverSocket);
    }
    if (mode == "CLIENT")
    {
        int conn = -1;
        do
        {
            conn = connect(serverSocket, (sockaddr *)&hint, sizeof(hint));
            // std::cout << "CONNECTION VALUE: " << conn << std::endl;
        } while (conn != 0);
    }
}

void CALFClientServer::SetupInitialKilobotStates()
{
    m_vecKilobotStates_ALF.resize(m_tKilobotEntities.size());
    m_vecKilobotsPositions.resize(m_tKilobotEntities.size());
    m_vecKilobotsColours.resize(m_tKilobotEntities.size());
    m_vecKilobotsOrientations.resize(m_tKilobotEntities.size());
    m_vecLastTimeMessaged.resize(m_tKilobotEntities.size());
    m_fMinTimeBetweenTwoMsg = Max<Real>(1.0, m_tKilobotEntities.size() * m_fTimeForAMessage / 3.0);

    /* Compute the number of kilobots on the field*/
    for (UInt16 it = 0; it < m_tKilobotEntities.size(); it++)
    {
        SetupInitialKilobotState(*m_tKilobotEntities[it]);
    }

    /* Initialization of kilobots variables */
    m_vecKilobotsTimer = std::vector<UInt8>(m_tKilobotEntities.size(), 0);
}

void CALFClientServer::SetupInitialKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
    m_vecLastTimeMessaged[unKilobotID] = -1000;

    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = argos::CColor::BLACK;
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
}

void CALFClientServer::SetupVirtualEnvironments(TConfigurationNode &t_tree)
{
}

void CALFClientServer::GetExperimentVariables(TConfigurationNode &t_tree)
{
    TConfigurationNode &tExperimentVariablesNode = GetNode(t_tree, "variables");
    GetNodeAttribute(tExperimentVariablesNode, "kilo_filename", m_strKiloOutputFileName);
    GetNodeAttribute(tExperimentVariablesNode, "area_filename", m_strAreaOutputFileName);
    GetNodeAttribute(tExperimentVariablesNode, "task_filename", m_strTaskOutputFileName);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "dataacquisitionfrequency", m_unDataAcquisitionFrequency, m_unDataAcquisitionFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "m_unEnvironmentPlotUpdateFrequency", m_unEnvironmentPlotUpdateFrequency, m_unEnvironmentPlotUpdateFrequency);
    GetNodeAttributeOrDefault(tExperimentVariablesNode, "timeforonemessage", m_fTimeForAMessage, m_fTimeForAMessage);
}

void CALFClientServer::UpdateKilobotState(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    m_vecKilobotsPositions[unKilobotID] = GetKilobotPosition(c_kilobot_entity);
    m_vecKilobotsOrientations[unKilobotID] = GetKilobotOrientation(c_kilobot_entity);
    m_vecKilobotsColours[unKilobotID] = GetKilobotLedColor(c_kilobot_entity);

    bool found = false;

    /* State transition*/
    if (initialised == true)
    {

        // here you manage in which area is the kilobot
        for (int i = 0; i < multiArea.size(); i++)
        {

            if (multiArea[i]->completed)
            {
                multiArea[i]->kilobots_in_area.clear();
                continue;
            }

            // inside bigger radius (radius)
            if (multiArea[i]->isInside(m_vecKilobotsPositions[unKilobotID]))
            {

                if (m_vecKilobotsColours[unKilobotID] == argos::CColor::RED)
                {
                    m_vecKilobotStates_ALF[unKilobotID] = LEAVING;
                    // std::cerr << "Leaving kID: " << unKilobotID << std::endl;
                    multiArea[i]->kilobots_in_area.erase(std::remove(multiArea[i]->kilobots_in_area.begin(), multiArea[i]->kilobots_in_area.end(), unKilobotID),
                                                         multiArea[i]->kilobots_in_area.end());

                    found = true;
                    break;
                }

                else
                {
                    if (std::find(multiArea[i]->kilobots_in_area.begin(), multiArea[i]->kilobots_in_area.end(), unKilobotID) == multiArea[i]->kilobots_in_area.end())
                        multiArea[i]->kilobots_in_area.push_back(unKilobotID);

                    m_vecKilobotStates_ALF[unKilobotID] = INSIDE_AREA;
                    m_vecKilobotsTimer[unKilobotID] = multiArea[i]->waiting_timer / 10;

                    found = true;
                    break;
                }
            }
            // outside
            // NOTE: if kilobot passes from blue to black you should remove it from the area position
            // if not in this area remove unKilobotID from it
            else
            {
                multiArea[i]->kilobots_in_area.erase(std::remove(multiArea[i]->kilobots_in_area.begin(), multiArea[i]->kilobots_in_area.end(), unKilobotID),
                                                     multiArea[i]->kilobots_in_area.end());
            }

            // if outside all areas
            if (!found)
            {
                m_vecKilobotStates_ALF[unKilobotID] = OUTSIDE_AREAS;
            }

            // /* LOG PRINT */
            // std::cout << "Robots in area " << multiArea[i]->id << ": ";
            // for (auto id : multiArea[i]->kilobots_in_area)
            // {
            //     std::cout << id << ", ";
            // }
            // std::cout << std::endl;
        }
        // switch (m_vecKilobotStates_ALF[unKilobotID]) {
        //     case OUTSIDE_AREAS : {
        //         std::cout<<"Outside\n";
        //     break;
        //     }
        //     case INSIDE_AREA : {
        //         std::cout<<"Inside\n";
        //     break;
        //     }
        //     case LEAVING : {
        //         std::cout<<"Leaving\n";
        //     break;
        //     }
        //     default:
        //         std::cout<<"Error no state";
        //     break;
        // }
    }
}

CVector2 CALFClientServer::VectorRotation2D(Real angle, CVector2 vec)
{
    Real kx = (cos(angle) * vec.GetX()) + (-1.0 * sin(angle) * vec.GetY());
    Real ky = (sin(angle) * vec.GetX()) + (cos(angle) * vec.GetY());
    CVector2 rotated_vector(kx, ky);
    return rotated_vector;
}

std::vector<int> CALFClientServer::Proximity_sensor(CVector2 obstacle_direction, Real kOrientation, int num_sectors)
{
    double sector = M_PI_2 / (num_sectors / 2.0);
    std::vector<int> proximity_values;

    for (int i = 0; i < num_sectors; i++)
    {
        CVector2 sector_dir_a = VectorRotation2D((kOrientation + M_PI_2 - i * sector), left_direction);
        CVector2 sector_dir_b = VectorRotation2D((kOrientation + M_PI_2 - (i + 1) * sector), left_direction);

        if (obstacle_direction.DotProduct(sector_dir_a) >= 0.0 || obstacle_direction.DotProduct(sector_dir_b) >= 0.0)
        {
            proximity_values.push_back(0);
        }
        else
        {
            proximity_values.push_back(1);
        }
    }

    return proximity_values;
}

void CALFClientServer::UpdateVirtualSensor(CKilobotEntity &c_kilobot_entity)
{
    UInt16 unKilobotID = GetKilobotId(c_kilobot_entity);
    // /********* WALL AVOIDANCE STUFF *************/
    UInt8 proximity_sensor_dec = 0; // 8 bit proximity sensor as decimal

    // std::cerr<<unKilobotID<<'\t'<<m_vecKilobotsPositions[unKilobotID]<<std::endl;
    if (fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > vDistance_threshold ||
        fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > vDistance_threshold)
    {
        std::vector<int> proximity_vec;

        if (m_vecKilobotsPositions[unKilobotID].GetX() > vDistance_threshold)
        {
            // std::cerr<<"RIGHT\n";
            proximity_vec = Proximity_sensor(right_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
        }
        else if (m_vecKilobotsPositions[unKilobotID].GetX() < -1.0 * vDistance_threshold)
        {
            // std::cerr<<"LEFT\n";
            proximity_vec = Proximity_sensor(left_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
        }

        if (m_vecKilobotsPositions[unKilobotID].GetY() > vDistance_threshold)
        {
            // std::cerr<<"UP\n";
            if (proximity_vec.empty())
                proximity_vec = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                proximity_vec = elementwiseOr;
            }
        }
        else if (m_vecKilobotsPositions[unKilobotID].GetY() < -1.0 * vDistance_threshold)
        {
            // std::cerr<<"DOWN\n";
            if (proximity_vec.empty())
                proximity_vec = Proximity_sensor(down_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
            else
            {
                std::vector<int> prox = Proximity_sensor(up_direction, m_vecKilobotsOrientations[unKilobotID].GetValue(), kProximity_bits);
                std::vector<int> elementwiseOr;
                elementwiseOr.reserve(prox.size());
                std::transform(proximity_vec.begin(), proximity_vec.end(), prox.begin(), std::back_inserter(elementwiseOr), std::logical_or<>());

                proximity_vec = elementwiseOr;
            }
        }

        proximity_sensor_dec = std::accumulate(proximity_vec.begin(), proximity_vec.end(), 0, [](int x, int y)
                                               { return (x << 1) + y; });
        // To turn off the wall avoidance decomment this
        // proximity_sensor_dec = 0;

        /** Print proximity values */
        // std::cerr<<"kID:"<< unKilobotID <<" sensor ";
        // for(int item : proximity_vec)
        // {
        //     std::cerr<< item <<'\t';
        // }
        // std::cerr<<std::endl;

        // std::cout<<"******Prox dec: "<<proximity_sensor_dec<<std::endl;
    }

    m_tALFKilobotMessage tKilobotMessage, tEmptyMessage, tMessage;
    bool bMessageToSend = false;

    if (m_fTimeInSeconds - m_vecLastTimeMessaged[unKilobotID] < m_fMinTimeBetweenTwoMsg)
    {
        return;
    }
    else
    {
        /* Compose the message for a kilobot */
        tKilobotMessage.m_sID = unKilobotID;                                // ID of the receiver
        tKilobotMessage.m_sType = (int)m_vecKilobotStates_ALF[unKilobotID]; // state
        tKilobotMessage.m_sData = 0;

        // entry msg when random walking
        if ((m_vecKilobotsColours[unKilobotID] != CColor::BLUE) &&
            (m_vecKilobotsColours[unKilobotID] != CColor::RED) &&
            ((int)m_vecKilobotStates_ALF[unKilobotID] == INSIDE_AREA))
        {
            bMessageToSend = true;
            tKilobotMessage.m_sData = (m_vecKilobotsTimer[unKilobotID] & 0xFF); // requirement (timer) for the area where it is
        }

        else if (proximity_sensor_dec)
        {
            bMessageToSend = true;
            tKilobotMessage.m_sData = proximity_sensor_dec;
            // std::cerr<<"sending COLLIDING\n";
        }

        // exit msg when inside
        else if ((m_vecKilobotsColours[unKilobotID] == CColor::RED || m_vecKilobotsColours[unKilobotID] == CColor::BLUE) &&
                 ((int)m_vecKilobotStates_ALF[unKilobotID] == OUTSIDE_AREAS))
        {
            bMessageToSend = true;
            // std::cerr<<"sending outside\n";
        }

        // /* Wall avoidance stuff */
        // if ((fabs(m_vecKilobotsPositions[unKilobotID].GetX()) > vDistance_threshold || fabs(m_vecKilobotsPositions[unKilobotID].GetY()) > vDistance_threshold) &&
        //     ((int)m_vecKilobotStates_ALF[unKilobotID] != INSIDE_AREA))
        // {
        //     tKilobotMessage.m_sData = proximity_sensor_dec;
        //     bMessageToSend = true;
        //     // std::cerr<<"sending COLLIDING\n";
        // }
        // bMessageToSend=true;      //use this line to send msgs always
    }

    if (bMessageToSend /*&& mode == "CLIENT"*/)
    {

        m_vecLastTimeMessaged[unKilobotID] = m_fTimeInSeconds;

        for (int i = 0; i < 9; ++i)
        {
            m_tMessages[unKilobotID].data[i] = 0;
        }
        tEmptyMessage.m_sID = 1023;
        tEmptyMessage.m_sType = 0;
        tEmptyMessage.m_sData = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (i == 0)
            {
                tMessage = tKilobotMessage;
            }
            else
            {
                tMessage = tEmptyMessage;
            }
            /* Packing the message */
            m_tMessages[unKilobotID].data[i * 3] = (tMessage.m_sID >> 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = (tMessage.m_sID << 6);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sType << 2);
            m_tMessages[unKilobotID].data[1 + i * 3] = m_tMessages[unKilobotID].data[1 + i * 3] | (tMessage.m_sData >> 8);
            m_tMessages[unKilobotID].data[2 + i * 3] = tMessage.m_sData;
            // std::cout<<" robot "<<tMessage.m_sID<<" "<<tMessage.m_sType<<std::endl;
        }
        // std::cout<<"payload: "<<tKilobotMessage.m_sData<<std::endl;
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, &m_tMessages[unKilobotID]);
    }
    else
    {
        GetSimulator().GetMedium<CKilobotCommunicationMedium>("kilocomm").SendOHCMessageTo(c_kilobot_entity, NULL);
    }
}

void CALFClientServer::PostStep()
{
    /* Listen for the other ALF communication */
    memset(inputBuffer, 0, 30);

    if (mode == "SERVER")
    {
        bytesReceived = recv(clientSocket, inputBuffer, 30, MSG_DONTWAIT);
    }
    else if (mode == "CLIENT")
    {
        bytesReceived = recv(serverSocket, inputBuffer, 30, MSG_DONTWAIT);
    }

    if ((bytesReceived != -1) || (bytesReceived != 0))
    {
        /* Save the received string in a vector, for having data available until next message comes */
        for (int i = 0; i < bytesReceived; i++)
        {
            storeBuffer[i] = inputBuffer[i];
        }
    }

    // // Print received message
    // // std::cerr << "Recv_str " << storeBuffer << std::endl;

    /* --------- CLIENT --------- */
    if (mode == "CLIENT")
    {
        /****RECEIVE PART*************************************************************************************************************************************/
        /* Initialize the tasks selected by the server */
        if ((storeBuffer[0] == 73) && (initialised == false))
        { // 73 is the ASCII binary for "I"
            /*choice of areas*/

            double white_space = 2.0 * kKiloDiameter;
            double radius = (((vArena_size * kScaling - white_space) / 4.0) - white_space) / 2.0;
            // std::cout << "radius:" << radius << std::endl;
            CVector2 areasOffset(-0.5, -0.5);

            std::string storebuffer(storeBuffer);
            storebuffer.erase(storebuffer.begin());
            desired_num_of_areas = storebuffer.size() / 3;
            std::string server_task(storebuffer.begin() + desired_num_of_areas, storebuffer.begin() + 2 * desired_num_of_areas);
            std::string client_task(storebuffer.begin() + 2 * desired_num_of_areas, storebuffer.end());
            // std::cout << "server_task:" << server_task << std::endl;
            // std::cout << "client_task:" << client_task << std::endl;
            // std::cout << "CLIENT - num of areas: " << desired_num_of_areas << std::endl;

            std::vector<int> active_areas;
            for (int i = 0; i < desired_num_of_areas; i++)
            {
                active_areas.push_back(storebuffer[i] - 97);
                // std::cout << "storebuffer[i] "<< storebuffer[i] << std::endl;
            }

            std::cout << "CLIENT - Active areas: \n";
            for (int id : active_areas)
            {
                std::cout << id << '\t';
            }
            std::cout << std::endl;
            // TODO : fix max_id, maybe in the namespace on top of this file
            for (int areaID = 0; areaID < 16; areaID++)
            {
                if (std::find(active_areas.begin(), active_areas.end(), areaID) != active_areas.end())
                {
                    CVector2 areaPos((1.0 + 2.0 * (areaID % 4)) * radius + (1.0 + (areaID % 4)) * white_space, (1.0 + floor(areaID / 4) * 2.0) * radius + (1.0 + floor(areaID / 4)) * white_space);
                    areaPos += areasOffset;
                    // std::cout << "areaPos " << areaID << ": " << areaPos << std::endl;
                    if (client_task.at(0) == '1')
                    {
                        if (server_task.at(0) == '1')
                            multiArea.push_back(new Area(areaID, HARD_TASK, HARD_TASK, areaPos, radius, vHardRequiredKilobots));
                        else
                            multiArea.push_back(new Area(areaID, HARD_TASK, SOFT_TASK, areaPos, radius, vHardRequiredKilobots));
                    }

                    else
                    {
                        if (server_task.at(0) == '1')
                            multiArea.push_back(new Area(areaID, SOFT_TASK, HARD_TASK, areaPos, radius, vSoftRequiredKilobots));
                        else
                            multiArea.push_back(new Area(areaID, SOFT_TASK, SOFT_TASK, areaPos, radius, vSoftRequiredKilobots));
                    }

                    client_task.erase(client_task.begin());
                    server_task.erase(server_task.begin());
                }
            }

            // std::cout << "multiArea.size" << multiArea.size() << std::endl;

            // std::cout<<"Recv_str "<<storeBuffer<<std::endl;
            initialised = true;
        }

        /* Align to server arena */
        if ((storeBuffer[0] == 65) && (initialised == true)) // 65 is the ASCII binary for "A"
        {
            // std::cout<<storeBuffer<<std::endl;
            for (int a = 0; a < multiArea.size(); a++)
            {
                int completed = storeBuffer[a + 1] - 48;
                if (completed != (multiArea[a]->completed == true ? 1 : 0))
                {
                    if (completed)
                    {
                        multiArea[a]->set_completed(m_fTimeInSeconds);
                        Area completed_area = *multiArea[a];
                        completed_areas.push_back(completed_area);

                        std::cout << "Completed area " << completed_area.id << " kIDs: ";
                        for (int i = 0; i < completed_area.kilobots_in_area.size(); i++)
                        {
                            std::cout << multiArea[a]->kilobots_in_area.at(i) << ", ";
                        }
                        std::cout << std::endl;
                    }
                    else
                    {
                        multiArea[a]->received_Respawn(m_fTimeInSeconds);
                    }
                }
            }
        }

        /****SENDING PART*************************************************************************************************************************************/
        std::string client_str;

        if (initialised == false)
        {
            client_str = "Missing parameters";
            // LOG print
            // std::cout << client_str << std::endl;
            send(serverSocket, client_str.c_str(), client_str.size() + 1, 0);
        }
        else if (storeBuffer[0] == 73) // 73 is the ASCII binary for "I"
        {
            client_str = "Received parameters";
            // LOG print
            // std::cout << client_str << std::endl;
            send(serverSocket, client_str.c_str(), client_str.size() + 1, 0);
        }
        else
        {
            /* Build the message for the other ALF */
            outputBuffer = "T"; //"T" indicates that the message is related to task completeness
            for (int k = 0; k < multiArea.size(); k++)
            {
                // /* LOG_PRINT */
                // std::cout << "Area:" << multiArea[k]->id << "kilo size:" << multiArea[k]->kilobots_in_area.size() << "task_requirement:" << multiArea[k]->task_requirement << std::endl;
                if (multiArea[k]->isReady())
                {
                    outputBuffer.append("1"); // hard task completed
                }
                else
                {
                    outputBuffer.append("0");
                }
            }

            // LOG print
            // std::cout << outputBuffer << std::endl;
            send(serverSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }
    }

    /* Send the message to the other ALF*/
    if (mode == "SERVER")
    {
        /****RECEIVE PART*************************************************************************************************************************************/
        /* Waiting for the Ack from the client */
        if (storeBuffer[0] == 82)
        {
            // std::cout << "ACK init by client*********\n";
            initialised = true;
        }

        /* Reactivation areas check */
        for (int i = 0; i < desired_num_of_areas; i++)
        {
            if (multiArea[i]->completed == true && multiArea[i]->Respawn(m_fTimeInSeconds))
            {
                multiArea[i]->received_Respawn(m_fTimeInSeconds);
            }
        }

        if (storeBuffer[0] == 84) // 84 is the ASCII binary for "T"
        {
            for (int j = 0; j < multiArea.size(); j++)
            {
                int ready_i = storeBuffer[j + 1] - 48;
                // /* LOG_PRINT */
                // std::cout << "Area:" << multiArea[j]->id << "kilo size:" << multiArea[j]->kilobots_in_area.size() << "task_requirement:" << multiArea[j]->task_requirement << std::endl;
                if (multiArea[j]->isCompleted(ready_i) && multiArea[j]->completed == false)
                {
                    multiArea[j]->set_completed(m_fTimeInSeconds);
                    Area completed_area = *multiArea[j];
                    completed_areas.push_back(completed_area);

                    // LOG print
                    std::cout << "Completed area " << completed_area.id << " kIDs: ";
                    for (int i = 0; i < completed_area.kilobots_in_area.size(); i++)
                    {
                        std::cout << multiArea[j]->kilobots_in_area.at(i) << ", ";
                    }
                    std::cout << std::endl;
                }
            }
        }
        /****SENDING PART*************************************************************************************************************************************/
        if (initialised == false)
        {
            // LOG print
            // std::cout << initialise_buffer << std::endl;
            send(clientSocket, initialise_buffer.c_str(), initialise_buffer.size() + 1, 0);
        }
        else
        {
            /* Build the message for the other ALF */
            outputBuffer = "A";
            for (int k = 0; k < desired_num_of_areas; k++)
            {
                if (multiArea[k]->completed == true)
                {
                    outputBuffer.append("1");
                }
                else
                {
                    outputBuffer.append("0");
                }
            }

            // LOG print
            // std::cout << outputBuffer << std::endl;
            send(clientSocket, outputBuffer.c_str(), outputBuffer.size() + 1, 0);
        }
    }

    if (!completed_areas.empty())
        CompletedAreasLOG();

    // std::cout << "Time: " << m_fTimeInSeconds << std::endl;
    internal_counter += 1;
    if (internal_counter % m_unDataAcquisitionFrequency == 0 || internal_counter <= 1)
    {
        KiloLOG();
        AreaLOG();
    }
}

void CALFClientServer::CompletedAreasLOG()
{
    while (!completed_areas.empty())
    {
        Area c_area = completed_areas.front();

        m_taskOutput
            << std::noshowpos
            << std::setw(8) << std::setprecision(4) << std::setfill('0')
            << m_fTimeInSeconds << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << c_area.id << '\t'
            << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << c_area.creation_time << '\t'
            << std::internal << std::noshowpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << c_area.completed_time << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << (c_area.color == argos::CColor::RED ? 1 : 0) << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << c_area.kilobots_in_area.size() << '\t';

        for (int i = 0; i < c_area.kilobots_in_area.size(); i++)
        {
            m_taskOutput << c_area.kilobots_in_area.at(i);
            if (i < c_area.kilobots_in_area.size() - 1)
            {
                m_taskOutput << ",";
            }
        }
        m_taskOutput << std::endl;

        completed_areas.erase(completed_areas.begin());
    }
}

void CALFClientServer::AreaLOG()
{
    // std::cerr << "Logging arePosition\n";
    m_areaOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds << '\t';
    for (size_t areaID = 0; areaID < multiArea.size(); areaID++)
    {
        m_areaOutput
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << multiArea[areaID]->id << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << multiArea[areaID]->position.GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << multiArea[areaID]->position.GetY() << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << (multiArea[areaID]->color == argos::CColor::RED ? 1 : 0) << '\t'
            << (multiArea[areaID]->completed == true ? 1 : 0) << '\t'
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << multiArea[areaID]->kilobots_in_area.size() << '\t';
        // << multiArea[areaID].
    }
    m_areaOutput << std::endl;
}

void CALFClientServer::KiloLOG()
{
    // std::cerr << "Logging kiloPosition\n";
    m_kiloOutput
        << std::noshowpos << std::setw(4) << std::setprecision(0) << std::setfill('0')
        << m_fTimeInSeconds << '\t';
    for (size_t kID = 0; kID < m_vecKilobotsPositions.size(); kID++)
    {
        m_kiloOutput
            // << kID << '\t'
            // << m_vecKilobotStates_ALF[kID] << '\t' //TODO: this should be the colour, but for now is the state
            // << m_vecKilobotsPositions[kID].GetX() << '\t'
            // << m_vecKilobotsPositions[kID].GetY() << '\t'
            // << m_vecKilobotsOrientations[kID] << '\t'
            // << m_vecKilobotStates_ALF[kID];

            // << std::noshowpos
            << std::noshowpos << std::setw(2) << std::setprecision(0) << std::setfill('0')
            << kID << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << m_vecKilobotsColours[kID] << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetX() << '\t'
            << std::internal << std::showpos << std::setw(8) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsPositions[kID].GetY() << '\t'
            << std::internal << std::showpos << std::setw(6) << std::setprecision(4) << std::setfill('0') << std::fixed
            << m_vecKilobotsOrientations[kID].GetValue() << '\t'
            << std::noshowpos << std::setw(1) << std::setprecision(0)
            << m_vecKilobotStates_ALF[kID] << '\t';
    }
    m_kiloOutput << std::endl;
}

CColor CALFClientServer::GetFloorColor(const CVector2 &vec_position_on_plane)
{
    CColor cColor = CColor::WHITE;

    /* Draw areas until they are needed, once that task is completed the corresponding area disappears */

    for (int i = 0; i < multiArea.size(); i++)
    {
        if (multiArea[i]->completed == false)
        {
            Real fDistance = Distance(vec_position_on_plane, multiArea[i]->position);
            if (fDistance < multiArea[i]->radius)
            {
                cColor = multiArea[i]->color;
            }
        }
    }

    // /** Center point */
    // Real fDistance00 = Distance(vec_position_on_plane, CVector2());
    // if (fDistance00 < 0.02)
    // {
    //     cColor = CColor::BLACK;
    // }

    return cColor;
}

REGISTER_LOOP_FUNCTIONS(CALFClientServer, "kilobot_ALF_dhtf_loop_function")