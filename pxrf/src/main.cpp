#include <QCoreApplication>
#include "vantacommunicator.h"

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    VantaCommunicator vc(argc, argv);
    vc.start(&app);
}
