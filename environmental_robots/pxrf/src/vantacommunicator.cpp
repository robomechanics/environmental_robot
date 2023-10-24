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
    chemistry_pub = n.advertise<pxrf::PxrfMsg>("pxrf_data", 1000);
    response_pub = n.advertise<std_msgs::String>("pxrf_response", 1000);
    ctrl_sub = n.subscribe("pxrf_cmd", 1000, &VantaCommunicator::callback, this);
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
    std::cout << "MSG: " << msg->data << std::endl;
    if (msg->data == "start" && !isRunning)
    {
        // isRunning = true;
        std::string startTestMessage = m_vantaMessageFactory.CreateStartTestMessage();
        std::cout << "Sending a Start Test Message " << std::endl << startTestMessage << std::endl;
        std::cout << "Now starting a test... " << std::endl;
        m_vantaConnection.sendToVanta(startTestMessage);
    }
    else if (msg->data == "stop" && isRunning) 
    {
        // isRunning = false;
        std::string stopTestMessage = m_vantaMessageFactory.CreateStopTestMessage();
        std::cout << "Sending a Stop Test Message " << std::endl << stopTestMessage << std::endl;
        std::cout << "Stopping test... " << std::endl;
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
        std::cout << "Got a Login response back from Vanta. " << response << std::endl;
        sleep(2);//sleeps for 2 second
        break;
    }

    case MessageFactory::StartTest:
        break;

    case MessageFactory::Notification:
        std::cout << "ID: " << id << std::endl;
        switch(id) {
            case MessageFactory::SystemStatus: {
                m_vantaMessageFactory.parseSystemStatusNotification(params, &systemStatus, &info);
                std::cout << "System status message: " << systemStatus << std::endl;
                if (info.length() > 0)
                    std::cout << "System status info: " << info << std::endl;
                break;
            }
            case MessageFactory::ResultReceived: {
                std::string chemistry;
                int dailyId;
                int testId;
                std::string testDateTime;
                m_vantaMessageFactory.parseForChemistry(params, &chemistry);
                m_vantaMessageFactory.parseForTimestamp(params, &dailyId, &testId, &testDateTime);
                std::cout <<chemistry << std::endl;
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
        std::cout << "Some response: " << response << std::endl;
        break;
    }
}

void VantaCommunicator::status(std::string status)
{
    std::cout << "Status: " << status << std::endl;

    if (status.compare(std::string("ok"))==0) {

        std::cout << "Established a websocket connection with the Vanta." << std::endl;
        std::string loginMessage = m_vantaMessageFactory.CreateLoginMessage("Administrator","0000");

        std::cout << "Creating a Login message " << std::endl << loginMessage << std::endl;
        std::cout << "Logging in to a Vanta as Administrator... " << std::endl;
        sleep(3); //sleeps for 1 second
        m_vantaConnection.sendToVanta(loginMessage);

    } else {

        std::cout << "Websocket error " << status << std::endl;
    }
}

void VantaCommunicator::start(QCoreApplication *app)
{
    /* Set the communicator instance with the WebSocket client so that the status() and messageResponse()
     * can be called to provide the health information of the connection and the response from Vanta.
     */
    m_vantaConnection.setVcInstance(this);

    /* Now connect to the device using the OTG interface. */
    std::string deviceIpAddr("192.168.7.2");//"192.168.7.10"); // change from 2 to 10
    // std::string deviceIpAddr("10.10.1.109");//"192.168.7.10"); // newIP for wireless

    std::cout << "Connecting to Vanta over the OTG interface " << deviceIpAddr << std::endl;
    m_vantaConnection.connectToVanta(deviceIpAddr);

    /* Execute the Qt application event loop. */
    app->exec();
}
