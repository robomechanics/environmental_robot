#ifndef WEBSOCKETCLIENT_CPP
#define WEBSOCKETCLIENT_CPP
#include "websocketclient.h"
#include "vantacommunicator.h"

const QString WebSocketClient::vantaReady           = "Server ready to receive commands.";
const QString WebSocketClient::vantaBeingControlled = "Rejecting connection. Device being controlled by ";
const QString WebSocketClient::userLoggedIn         = "Rejecting connection. User is Logged In.";
const QString WebSocketClient::controllerMessage    = "Hi i am a controller";

WebSocketClient::WebSocketClient()
{
    m_connectionState = SockNotConnected;
    connect(&m_webSocket, &QWebSocket::textMessageReceived,     this, &WebSocketClient::onTextMessageReceived);
    connect(&m_webSocket, &QWebSocket::binaryMessageReceived,   this, &WebSocketClient::onBinaryMessageReceived);
    connect(&m_webSocket, &QWebSocket::connected,               this, &WebSocketClient::onConnected);
    connect(&m_webSocket, &QWebSocket::disconnected,            this, &WebSocketClient::onDisconnected);
    connect(&m_webSocket, static_cast<void (QWebSocket::*)(QAbstractSocket::SocketError)>(&QWebSocket::error), this, &WebSocketClient::socketError);
    m_webSocket.ignoreSslErrors();
    m_vcInstance = NULL;

    m_udpSocket.bind(XDD_UDP_BROADCAST_PORT, QUdpSocket::ShareAddress);

    m_watchdogTimer = new QTimer;
    m_watchdogTimer->setInterval(1000);
    connect(m_watchdogTimer, &QTimer::timeout, this, &WebSocketClient::sendHeartBeatMessageToDevice);
}

WebSocketClient::~WebSocketClient()
{
}

void WebSocketClient::setVcInstance(VantaCommunicator *vcInstance)
{
    m_vcInstance = vcInstance;
}

void WebSocketClient::sendToVanta(std::string cmd)
{
    sendBinaryMsgToVanta(cmd.data(), cmd.length());
}

void WebSocketClient::connectToVanta(std::string ipAddress)
{
    if (m_connectionState == SockNotConnected) {
        m_url = "ws://" + QString(ipAddress.data()) + ":" + QString::number(vantaWebSocketPortNumber);
        m_deviceIpAddress = ipAddress.data();
        m_webSocket.open(m_url);
        m_connectionState = SockEstablishingConnection;
    }
}

void WebSocketClient::closeVantaConnection()
{
    m_webSocket.close();
}

void WebSocketClient::sendHeartBeatMessageToDevice()
{
    if(m_connectionState == SockConnected ) {
        QByteArray sendDatagram;
        sendDatagram.append("MsgFromPC");
        qint64 rval = m_udpSocket.writeDatagram(sendDatagram.data(), sendDatagram.size(), QHostAddress(m_deviceIpAddress), XDD_UDP_BROADCAST_PORT);
        if(rval<0) {
            qWarning() << "WARN: UDP write failure";
        }
    }
}
void WebSocketClient::onConnected()
{
    if (m_connectionState == SockEstablishingConnection)
        sendTextMsgToVanta(controllerMessage.toLocal8Bit());
    m_watchdogTimer->start();
}

void WebSocketClient::onDisconnected()
{
    if (m_connectionState == SockConnected) {
        m_connectionState = SockNotConnected;
        m_url = QString();
    }
    m_watchdogTimer->stop();
}

void WebSocketClient::onTextMessageReceived(QString message)
{
    std::string statusMessage;

    if (message.contains(userLoggedIn)) {
        m_connectionState = SockNotConnected;
        m_webSocket.close();
        statusMessage = "Device is BUSY";
    }

    if (message.contains(vantaBeingControlled)) {
        m_connectionState = SockNotConnected;
        m_webSocket.close();
        statusMessage = "Device is BUSY";
    }

    if (message.contains(vantaReady)) {
        m_connectionState = SockConnected;
        statusMessage = "ok";
    }

    m_vcInstance->status(statusMessage);
}

void WebSocketClient::onBinaryMessageReceived(QByteArray message)
{
    m_vcInstance->messageResponse(message.toStdString());
}

void WebSocketClient::sendTextMsgToVanta(const char *message)
{
    m_webSocket.sendTextMessage(message);
}

void WebSocketClient::sendBinaryMsgToVanta(const char *message, quint32 sizeOfMessage)
{
    const QByteArray qBuffer(message, sizeOfMessage);
    m_webSocket.sendBinaryMessage(qBuffer);
}

void WebSocketClient::socketError(QAbstractSocket::SocketError error)
{
    QString sockErrString;
    switch(error) {
    case QAbstractSocket::ConnectionRefusedError:
        sockErrString = "Connection was refused";
        break;
    case QAbstractSocket::RemoteHostClosedError:
        sockErrString = "Remote Host Closed error";
        break;
    case QAbstractSocket::HostNotFoundError:
        sockErrString = "Host Not Found error";
        break;
    case QAbstractSocket::SocketAccessError:
        sockErrString = "Socket Access error";
        break;
    case QAbstractSocket::SocketResourceError:
        sockErrString = "Socket resource error";
        break;
    case QAbstractSocket::SocketTimeoutError:
        sockErrString = "Socket timeout error";
        break;
    case QAbstractSocket::DatagramTooLargeError:
        sockErrString = "Datagram too large error";
        break;
    case QAbstractSocket::NetworkError:
        sockErrString = "Datagram too large error";
        break;
    case QAbstractSocket::AddressInUseError:
        sockErrString = "Address in use error";
        break;
    case QAbstractSocket::SocketAddressNotAvailableError:
        sockErrString = "Socket address not available error";
        break;
    case QAbstractSocket::UnsupportedSocketOperationError:
        sockErrString = "Unsupported socket operation error";
        break;
    case QAbstractSocket::ProxyAuthenticationRequiredError:
        sockErrString = "Proxy authentication required error";
        break;
    case QAbstractSocket::SslHandshakeFailedError:
        sockErrString = "SSL Handshake failed error";
        break;
    case QAbstractSocket::UnfinishedSocketOperationError:
        sockErrString = "Unfinished socket operation error";
        break;
    case QAbstractSocket::ProxyConnectionRefusedError:
        sockErrString = "Proxy connection refused error";
        break;
    case QAbstractSocket::ProxyConnectionClosedError:
        sockErrString = "Proxy connection closed error";
        break;
    case QAbstractSocket::ProxyConnectionTimeoutError:
        sockErrString = "Proxy connection timeout error";
        break;
    case QAbstractSocket::ProxyNotFoundError:
        sockErrString = "Proxy not found error";
        break;
    case QAbstractSocket::ProxyProtocolError:
        sockErrString = "Proxy protocol error";
        break;
    case QAbstractSocket::OperationError:
        sockErrString = "Operation error";
        break;
    case QAbstractSocket::SslInternalError:
        sockErrString = "SSL Internal error";
        break;
    case QAbstractSocket::SslInvalidUserDataError:
        sockErrString = "SSL Invalid user data error";
        break;
    case QAbstractSocket::TemporaryError:
        sockErrString = "Temporary error";
        break;
    case QAbstractSocket::UnknownSocketError:
        sockErrString = "Unknown socket error";
        break;
    default:
        sockErrString = "Default socket error handler";
        break;
    }

    m_vcInstance->status(sockErrString.toStdString());
}

#endif // WEBSOCKETCLIENT_CPP

