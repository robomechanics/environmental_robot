#include "vantacommunicator.h"
#include <unistd.h>


VantaCommunicator::VantaCommunicator(int argc, char** argv)
{
    ros::init(argc, argv, "pxrf");
    ros::NodeHandle n("~");
    state = Startup;

    n.param<std::string>("vanta_ip", vanta_ip, "192.168.7.2");

    ctrl_sub = n.subscribe("cmd", 10, &VantaCommunicator::command_callback, this);
    chemistry_pub = n.advertise<pxrf::PxrfMsg>("data", 10);
    response_pub = n.advertise<std_msgs::String>("response", 10);
    state_pub = n.advertise<std_msgs::String>("state", 1);

    QTimer *stateTimer = new QTimer(this);
    connect(stateTimer, SIGNAL(timeout()), this, SLOT(publishState()));
    stateTimer->start(1000);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(petWatchdog()));
    timer->start(1000);

    std::string pxrf_cmd_topic, pxrf_data_topic, pxrf_response_topic;
    n.getParam("pxrf_cmd_topic", pxrf_cmd_topic);
    n.getParam("pxrf_data_topic", pxrf_data_topic);
    n.getParam("pxrf_response_topic", pxrf_response_topic);
    
    ctrl_sub = n.subscribe(pxrf_cmd_topic, 1000, &VantaCommunicator::callback, this);
    chemistry_pub = n.advertise<pxrf::PxrfMsg>(pxrf_data_topic, 1000);
    response_pub = n.advertise<std_msgs::String>(pxrf_response_topic, 1000);
}

VantaCommunicator::~VantaCommunicator()
{
    ros::shutdown();
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

void VantaCommunicator::command_callback(const std_msgs::String::ConstPtr& msg)
{
    // ROS_INFO("Received command: %s", msg->data.c_str());
    if (msg->data == "start" && state != Reading)
    {
        // isRunning = true;
        std::string startTestMessage = m_vantaMessageFactory.CreateStartTestMessage();
        ROS_INFO("Sending a Start Test Message");
        m_vantaConnection.sendToVanta(startTestMessage);
    }
    else if (msg->data == "stop" && state == Reading) 
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
    std::string error, params;
    m_vantaMessageFactory.parseMessageResponse(response, &messageId, &id, &error, &params);
    ros::spinOnce();
    
    switch(messageId) {
    case MessageFactory::Login: {
        ROS_INFO("Got a Login response back from Vanta");
        if (state == Startup) {
            state = Ready;
            ROS_INFO("Transitioning to Ready state");
        }
        sleep(2);
        break;
    }

    case MessageFactory::StartTest:
        break;

    case MessageFactory::Notification:
        // std::cout << id << std::endl;
        switch(id) {
            case MessageFactory::SystemStatus: {
                std::string systemStatus, info;
                m_vantaMessageFactory.parseSystemStatusNotification(params, &systemStatus, &info);
                if (systemStatus == "Ready")
                {
                    ROS_INFO_ONCE("Ready");
                }
                else{
                    ROS_INFO_ONCE("Status:  %s", systemStatus.c_str());
                }
                
                if (info.length() > 0)
                    ROS_INFO_THROTTLE(5, "%s", info.c_str());
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
                ROS_INFO("Test started");
                std_msgs::String msg;
                msg.data = "200";
                response_pub.publish(msg);
                state = Reading;
                break;
            }
            case MessageFactory::TestStopped: {
                ROS_INFO("Test stopped");
                std_msgs::String msg;
                msg.data = "201";
                response_pub.publish(msg);
                state = Ready;
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
    if (status.compare(std::string("ok")) == 0) {
        ROS_INFO("Established a websocket connection with the Vanta");

        ROS_INFO("Logging in to a Vanta as Administrator...");
        std::string loginMessage = m_vantaMessageFactory.CreateLoginMessage("Administrator","0000");
        sleep(3);
        m_vantaConnection.sendToVanta(loginMessage);

    } else {
        ROS_ERROR("Websocket client error: %s", status.c_str());
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

void VantaCommunicator::publishState()
{
    ros::spinOnce();
    std_msgs::String msg;
    switch (state) {
        case Startup:
            msg.data = "STARTUP";
            break;
        case Ready:
            msg.data = "READY";
            break;
        case Reading:
            msg.data = "READING";
            break;
    }
    state_pub.publish(msg);
}
