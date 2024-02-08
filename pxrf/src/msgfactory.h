#ifndef CMDFACTORY_H
#define CMDFACTORY_H
#include <iostream>
#include <string>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

/* MessageFactory is a typical Factory class that creates various messages that can then be sent over a websocket interface
 * to a Vanta.
 */
class MessageFactory
{
public:
    enum VantaMessages
    {
        PetWatchdog = 244,
        Login = 301,
        StartTest = 601,
        StopTest = 602,
        Notification = 403
    };

    enum VantaNotifications
    {
        SystemStatus = 100,
        TestStarted = 200,
        TestStopped = 201,
        ResultReceived = 206
    };

    MessageFactory();
    ~MessageFactory();

    /* Creates a message to pet the watchdog.
     * An example of the Pet Watchdog Message is shown below.
     *       {
     *           "commandId": 244,
     *           "id": 1,
     *           "params": {
     *                     }
     *       }
     */
    std::string CreatePetWatchdogMessage();

    /* Creates a Login message based upon the login and password provided.
     * All messages to the Vanta and responses received from the Vanta are JSON objects.
     * An example of the Create Login Message is shown below
     *       {
     *          "commandId": 301,
     *          "id": 1,
     *          "params": {
     *              "password": "0000",
     *              "userId": "Administrator"
     *          }
     *       }
     */
    std::string CreateLoginMessage(std::string login, std::string password);

    /* Creates a start test message.
     * An example of the Start Test Message is shown below.
     *       {
     *           "commandId": 601,
     *           "id": 1,
     *           "params": {
     *                     }
     *       }
     */
    std::string CreateStartTestMessage();

    /* Creates a stop test message.
     * An example of the Start Test Message is shown below.
     *       {
     *           "commandId": 602,
     *           "id": 1,
     *           "params": {
     *                     }
     *       }
     */
    std::string CreateStopTestMessage();

    /* A simple utility that will parse a response provided by a Vanta into relavant pieces.
     * An example of a login response from the device is shown below.
     *       {
     *           "commandId": 601,
     *           "id": 1,
     *           "params": {
     *                     }
     *       }
     */
    void parseMessageResponse(std::string respStr, int *msgId, int *id, std::string *error, std::string *params);

    void parseForChemistry(std::string params, std::string *chemistry);

    void parseForTimestamp(std::string params,int *dailyId, int *testId, std::string *testDateTime);

    /* A simple utility to parse the notification response from Vanta. */
    void parseSystemStatusNotification(std::string ssNotification, std::string *systemStatus, std::string *info);
};

#endif // CMDFACTORY_H

