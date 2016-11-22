#include <iostream>

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlProperty>
#include <QObject>
#include <QDebug>
#include <QString>
#include <functional>

#include "XbeeInterface.hpp"
#include "../../NGCP/xbeeplus/include/SerialXbee.hpp"
#include "../../NGCP/xbeeplus/include/ReceivePacket.hpp"


int main(int argc, char *argv[])
{
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QGuiApplication app(argc, argv);

    // must register types before loading application
    qmlRegisterSingletonType<XbeeInterface>("XbeeInterface", 1, 0, "XbeeInterface", singleton_MessageHandler);

    QQmlApplicationEngine engine;
    engine.load(QUrl(QLatin1String("qrc:/main.qml")));

    return app.exec();
}
