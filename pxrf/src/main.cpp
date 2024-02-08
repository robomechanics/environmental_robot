#include <QCoreApplication>
#include "vantacommunicator.h"

int main(int argc, char *argv[])
{
    QCoreApplication sampleApp(argc, argv);
    VantaCommunicator vc(argc, argv);
    vc.start(&sampleApp);
}
