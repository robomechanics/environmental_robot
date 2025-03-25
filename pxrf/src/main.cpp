#include <QCoreApplication>
#include "vantacommunicator.h"
#include <signal.h>

// Signal handler function
void signalHandler(int signum) {
    QCoreApplication::quit();
}

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    VantaCommunicator vc(argc, argv);
    signal(SIGINT, signalHandler);
    vc.start(&app);
}
