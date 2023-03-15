#ifndef VANTACOMMUNICATOR_H
#define VANTACOMMUNICATOR_H
#include <QCoreApplication>
#include <QTimer>
#include "websocketclient.h"
#include "msgfactory.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pxrf/PxrfMsg.h"

/* A simple class that allows the application to open up a connection with a Vanta using the OTG interface.
 * When a healthy connection is established it allows the application to send various commands to the Vanta
 * and receive responses from it.
 */
class VantaCommunicator : public QObject
{

    Q_OBJECT

public:

    VantaCommunicator(int argc, char** argv);
    ~VantaCommunicator();

    bool isRunning;

    ros::Publisher chemistry_pub;
    ros::Publisher response_pub;
    ros::Subscriber ctrl_sub;

    // Timer object used to pet the watchdog
    QTimer *timer;

    /* No loop rate/sleep needed since it loops at speed defined by Qt */

    /* This method will be called by the Websocket interface when it has received some response from a Vanta.
     */
    void messageResponse(std::string response);

    /* This method will be called by the websocket interface when it has successfully established a connection with a Vanta
     * and it is ready to accept commands or if an error happens on the interface.
     */
    void status(std::string status);

    /* This will initiate a Websocket connection with a Vanta device. */
    void start(QCoreApplication *app);

    /* This will publish the chemistry result to ROS */
    void publishChemistry(std::string chemistry, int dailyId, int testId, std::string testDateTime);

    /* Callback function for commands from gui */
    void callback(const std_msgs::String::ConstPtr& msg);

public slots:
    /* This method will send the PetWatchdog message to the Vanta device. */
    void petWatchdog();

private:
    MessageFactory   m_vantaMessageFactory;
    WebSocketClient  m_vantaConnection;
};

#endif // VANTACOMMUNICATOR_H

