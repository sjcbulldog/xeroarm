#include "xeroarm.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QCoreApplication::setOrganizationName("ErrorCodeXero");
    QCoreApplication::setOrganizationDomain("www.wilsonvillerobotics.com");
    QCoreApplication::setApplicationName("XeroArmPlanner");
    QCoreApplication::setApplicationVersion("1.0.0");

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication a(argc, argv);
    xeroarm w;
    w.show();
    return a.exec();
}
