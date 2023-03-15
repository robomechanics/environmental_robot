#include "msgfactory.h"

MessageFactory::MessageFactory()
{
}

MessageFactory::~MessageFactory()
{
}

void MessageFactory::parseMessageResponse(std::string respStr, int *msgId, int *id, std::string *error, std::string *params)
{
    QString qrespStr = QString::fromUtf8(respStr.data());
    QJsonDocument jsDoc = QJsonDocument::fromJson(qrespStr.toUtf8());
    QJsonObject jsObj = jsDoc.object();
    *msgId = jsObj.value("commandId").toInt();
    *id = jsObj.value("id").toInt();
    if (!jsObj.value("error").isUndefined())
        *error = jsObj.value("error").toString().toStdString();
    QJsonObject paramsObj = jsObj.value("params").toObject();
    QJsonDocument paramsDoc(paramsObj);
    *params = paramsDoc.toJson().toStdString();
}

void MessageFactory::parseForChemistry(std::string params, std::string *chemistry)
{
    QString qparams = QString::fromStdString(params);
    QJsonDocument paramsDoc = QJsonDocument::fromJson(qparams.toUtf8());
    QJsonObject paramsObj = paramsDoc.object();
    QJsonObject resultObj = paramsObj.value("result").toObject();
    QJsonArray chemistryObj = resultObj.value("chemistry").toArray();
    QJsonDocument chemistryDoc(chemistryObj);
    *chemistry = chemistryDoc.toJson().toStdString();
}

void MessageFactory::parseForTimestamp(std::string params,int *dailyId, int *testId, std::string *testDateTime)
{
    QString qparams = QString::fromStdString(params);
    QJsonDocument paramsDoc = QJsonDocument::fromJson(qparams.toUtf8());
    QJsonObject paramsObj = paramsDoc.object();
    QJsonObject resultObj = paramsObj.value("result").toObject();
    QJsonObject analysisObj = resultObj.value("analysis").toObject();
    *dailyId = analysisObj.value("dailyId").toInt();
    *testId = analysisObj.value("testId").toInt();
    *testDateTime = analysisObj.value("testDateTime").toString().toStdString();
}

void MessageFactory::parseSystemStatusNotification(std::string ssNotification, std::string *systemStatus, std::string *info)
{
    QJsonDocument jsDoc = QJsonDocument::fromJson(ssNotification.data());
    QJsonObject jsObj = jsDoc.object();
    *systemStatus = jsObj.value("systemStatus").toString().toStdString();
    *info = jsObj.value("info").toString().toStdString();
}

std::string MessageFactory::CreatePetWatchdogMessage()
{
    /* Create and initialise the pet watchdog message. */
    QJsonObject cmdObj, paramsObj;
    cmdObj.insert("commandId", QJsonValue((int) PetWatchdog));
    cmdObj.insert("id", QJsonValue(1));
    cmdObj.insert("params", QJsonValue(paramsObj));
    QJsonDocument jsDoc(cmdObj);
    return(jsDoc.toJson().toStdString());
}

std::string MessageFactory::CreateLoginMessage(std::string login, std::string password)
{
    /* Create and initialise the parameters Json Object. */
    QJsonObject paramsObj;
    paramsObj.insert("userId", QJsonValue(login.data()));
    paramsObj.insert("password", QJsonValue(password.data()));

    /* Create and initialise the login message. */
    QJsonObject cmdObj;

    /* Provide the message id. */
    cmdObj.insert("commandId", QJsonValue((int) Login));

    /* Provide the id. Its a simple token that must be provided. This token is returned back
     * in the response.
     */
    cmdObj.insert("id", QJsonValue(1));

    /* Provide the parameters. */
    cmdObj.insert("params", QJsonValue(paramsObj));
    QJsonDocument jsDoc(cmdObj);
    return(jsDoc.toJson().toStdString());
}

std::string MessageFactory::CreateStartTestMessage()
{
    /* Create and initialise the start test message. */
    QJsonObject cmdObj, paramsObj;
    cmdObj.insert("commandId", QJsonValue((int) StartTest));
    cmdObj.insert("id", QJsonValue(1));
    cmdObj.insert("params", QJsonValue(paramsObj));
    QJsonDocument jsDoc(cmdObj);
    return(jsDoc.toJson().toStdString());
}

std::string MessageFactory::CreateStopTestMessage()
{
    /* Create and initialise the stop test message. */
    QJsonObject cmdObj, paramsObj;
    cmdObj.insert("commandId", QJsonValue((int) StopTest));
    cmdObj.insert("id", QJsonValue(1));
    cmdObj.insert("params", QJsonValue(paramsObj));
    QJsonDocument jsDoc(cmdObj);
    return(jsDoc.toJson().toStdString());
}
