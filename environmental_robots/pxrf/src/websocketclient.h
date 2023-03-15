#ifndef WEBSOCKETCLIENT_H
#define WEBSOCKETCLIENT_H

#include <QtCore/QObject>
#include <QtCore/QList>
#include <QtCore/QByteArray>
#include <QtCore/QString>
#include <QtCore/QUrl>
#include <QtNetwork/QSslError>
#include <QUdpSocket>
#include <QTimer>
#include <QMutex>
#include <iostream>
#include "QtWebSockets/QWebSocketServer"
#include "QtWebSockets/QWebSocket"

#define XDD_UDP_BROADCAST_PORT      7862

class VantaCommunicator;

/* The WebSocketClient is a wrapper around the Qt Websocket interface. It provides the ability to use
 * Qt websockets to communicate with a Vanta, send and receive text and binary messages.
 * It also informs the communicator of any issues on the websocket interface.
 */
class WebSocketClient : public QObject
{
    Q_OBJECT

public:
    WebSocketClient();
    ~WebSocketClient();

    /* Sets the Vanta communicator instance for relevant callbacks. */
    void setVcInstance(VantaCommunicator *instance);

    /* Sends a message to the device. */
    void sendToVanta(std::string command);

    /* Connects to the Vanta device with the given ip address. */
    void connectToVanta(std::string ipAddr);

    /* Closes the connection with the device. */
    void closeVantaConnection();

public slots:
    void sendHeartBeatMessageToDevice();

private slots:
    /* slot handlers for Qt signals to provide state change information. */
    void onConnected();
    void onDisconnected();
    void onTextMessageReceived(QString message);
    void onBinaryMessageReceived(QByteArray message);
    void socketError(QAbstractSocket::SocketError err);

private:
    /* Tracks the state of the websocket connection with the Vanta. */
    enum WebsocketConnectionState
    {
        SockNotConnected,
        SockEstablishingConnection,
        SockConnected
    };

    /* Control messages exchanged between the Vanta and the application. */
    static const QString vantaReady;
    static const QString vantaBeingControlled;
    static const QString userLoggedIn;
    static const QString controllerMessage;

    /* Vanta is listening on port number 7860 over websockets. */
    static const int vantaWebSocketPortNumber = 7860;

    void sendTextMsgToVanta(const char *message);
    void sendBinaryMsgToVanta(const char *, quint32);
    QWebSocket                  m_webSocket;
    QUrl                        m_url;
    QString                     m_deviceIpAddress;

    QUdpSocket          m_udpSocket;
    QTimer          *m_watchdogTimer;

    /* Tracks the state of the websocket connection. */
    WebsocketConnectionState    m_connectionState;

    /* Save the instance of the Vanta Communicator to notify responses. */
    VantaCommunicator          *m_vcInstance;
};

#endif // WEBSOCKETCLIENT_H

