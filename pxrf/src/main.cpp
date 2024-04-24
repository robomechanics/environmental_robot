#include <QCoreApplication>
#include "vantacommunicator.h"
#include <signal.h>

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    VantaCommunicator vc(argc, argv);
    signal(SIGINT, SIG_DFL);
    vc.start(&app);
}
