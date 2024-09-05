#include "vantacommunicator.h"
#include <unistd.h>

VantaCommunicator::VantaCommunicator(int argc, char** argv)
{
    ros::init(argc, argv, "pxrf");
    ros::NodeHandle n;
    isRunning = false;
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(petWatchdog()));
    timer->start(1000);

    std::string pxrf_cmd_topic, pxrf_data_topic, pxrf_response_topic;
    n.getParam("pxrf_cmd_topic", pxrf_cmd_topic);
    n.getParam("pxrf_data_topic", pxrf_data_topic);
    n.getParam("pxrf_response_topic", pxrf_response_topic);
    n.getParam("vanta_ip", vanta_ip);
    
    ctrl_sub = n.subscribe(pxrf_cmd_topic, 1000, &VantaCommunicator::callback, this);
    chemistry_pub = n.advertise<pxrf::PxrfMsg>(pxrf_data_topic, 1000);
    response_pub = n.advertise<std_msgs::String>(pxrf_response_topic, 1000);
}

VantaCommunicator::~VantaCommunicator()
{
}

void VantaCommunicator::petWatchdog()
{
    std::string petWatchdogMessage = m_vantaMessageFactory.CreatePetWatchdogMessage();
    m_vantaConnection.sendToVanta(petWatchdogMessage);
}

void VantaCommunicator::publishChemistry(std::string chemistry, int dailyId, int testId, std::string testDateTime)
{
    pxrf::PxrfMsg msg;
    msg.chemistry = chemistry;
    msg.dailyId = dailyId;
    msg.testId = testId;
    msg.testDateTime = testDateTime;
    chemistry_pub.publish(msg);
}

void VantaCommunicator::callback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << msg->data << std::endl;
    if (msg->data == "start" && !isRunning)
    {
        // isRunning = true;
        std::string startTestMessage = m_vantaMessageFactory.CreateStartTestMessage();
        ROS_INFO("Sending a Start Test Message");
        m_vantaConnection.sendToVanta(startTestMessage);
    }
    else if (msg->data == "stop" && isRunning) 
    {
        // isRunning = false;
        std::string stopTestMessage = m_vantaMessageFactory.CreateStopTestMessage();
        ROS_INFO("Sending a Stop Test Message");
        m_vantaConnection.sendToVanta(stopTestMessage);
    }
}

void VantaCommunicator::messageResponse(std::string response)
{
    int messageId, id;
    std::string error, params, systemStatus, info;
    m_vantaMessageFactory.parseMessageResponse(response, &messageId, &id, &error, &params);
    ros::spinOnce();
    
    switch(messageId) {
    case MessageFactory::Login: {
        ROS_INFO("Got a Login response back from Vanta");
        sleep(2);//sleeps for 2 second
        break;
    }

    case MessageFactory::StartTest:
        break;

    case MessageFactory::Notification:
        // std::cout << id << std::endl;
        switch(id) {
            case MessageFactory::SystemStatus: {
                m_vantaMessageFactory.parseSystemStatusNotification(params, &systemStatus, &info);
                if (systemStatus == "Ready")
                {
                    ROS_INFO_ONCE("Ready");
                }
                else{
                    ROS_INFO_ONCE("Status:  %s", systemStatus.c_str());
                }
                
                if (info.length() > 0)
                    ROS_INFO_THROTTLE(5, "%s", info.c_str());;
                break;
            }
            case MessageFactory::ResultReceived: {
                std::string chemistry;
                int dailyId;
                int testId;
                std::string testDateTime;
                m_vantaMessageFactory.parseForChemistry(params, &chemistry);
                m_vantaMessageFactory.parseForTimestamp(params, &dailyId, &testId, &testDateTime);
                ROS_INFO("Chemistry:  \n %s ---\n", chemistry.c_str());
                VantaCommunicator::publishChemistry(chemistry, dailyId, testId, testDateTime);
                break;
            }
            case MessageFactory::TestStarted: {
                std_msgs::String msg;
                msg.data = "200";
                response_pub.publish(msg);
                isRunning = true;
                break;
            }
            case MessageFactory::TestStopped: {
                std_msgs::String msg;
                msg.data = "201";
                response_pub.publish(msg);
                isRunning = false;
                break;
            }
            default: {
                break;
            }
        }
        break;

    default:
        // std::cout << "some response " << response << std::endl;
        break;
    }
}

void VantaCommunicator::status(std::string status)
{
    if (status.compare(std::string("ok"))==0) {

        ROS_INFO("Established a websocket connection with the Vanta");

        std::string loginMessage = m_vantaMessageFactory.CreateLoginMessage("Administrator","0000");

        // std::cout << "Creating a Login message " << std::endl << loginMessage << std::endl;
        ROS_INFO("Logging in to a Vanta as Administrator...");

        sleep(3); //sleeps for 1 second
        m_vantaConnection.sendToVanta(loginMessage);

    } else {
        ROS_WARN("Might be a Websocket error");
    }
}

void VantaCommunicator::start(QCoreApplication *app)
{
    /* Set the communicator instance with the WebSocket client so that the status() and messageResponse()
     * can be called to provide the health information of the connection and the response from Vanta.
     */
    m_vantaConnection.setVcInstance(this);

    /* Now connect to the device using the OTG interface. */
    std::string deviceIpAddr(vanta_ip);

    ROS_INFO("Connecting to Vanta over the OTG interface: %s", deviceIpAddr.c_str());

    m_vantaConnection.connectToVanta(deviceIpAddr);

    /* Execute the Qt application event loop. */
    app->exec();
}
