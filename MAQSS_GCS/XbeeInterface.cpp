#include "XbeeInterface.hpp"

XbeeInterface::XbeeInterface(QObject *parent) : QObject(parent)
{
    qDebug() << "Instantiating single_test class";
    serial_interface.ReadHandler = std::bind(&XbeeInterface::callbackFun, this, std::placeholders::_1);
    serial_interface.AsyncReadFrame();
}

void XbeeInterface::callbackFun(XBEE::Frame *item) {

    // ReceivePacket pointer
    // dynamic cast to type of frame I think it is (ReceivePacket), store in pointer
    XBEE::ReceivePacket *r_packet = dynamic_cast<XBEE::ReceivePacket*>(item);
    std::string str_data;

    qDebug() << "Entered Callback";

    // check if pointer is NULL
    if (r_packet != NULL) {
        str_data = r_packet->GetData();
        std::cout << "Data: " << str_data.c_str() << std::endl;
    }

    QString q_str = QString::fromUtf8(str_data.c_str());
    emit newMsg(q_str);

//    qDebug() << "Inside Here";
}

void XbeeInterface::writeMsg(QString msg) {
    // Assumes msg is of the form NEWMSG,TYPE,QX,....

    // Split string by , to determine target
    qDebug() << "Writing Msg: " << msg;
    QStringList list1 = msg.split(',', QString::SkipEmptyParts);
    QString target_vehicle = list1.at(2).at(1);
    unsigned int target_id = target_vehicle.toUInt();
    std::string str_data = msg.toStdString();

    // TODO: Make this less hardcoded
    XBEE::TransmitRequest frame_a(0x0013A20040A8157E);
    XBEE::TransmitRequest frame_b(0x0013A20040F8063C);
    XBEE::TransmitRequest frame_c(0x0013A20040F8064D);
    XBEE::TransmitRequest frame_d(0x0013A20040A815D6);

        qDebug() << "Target_id: " << target_id;
    // TODO: Refactor to not be hardcoded
    switch (target_id) {
    case 0:
        frame_a.SetData(str_data);
        serial_interface.AsyncWriteFrame(&frame_a);
        break;

    case 1:
        frame_b.SetData(str_data);
        serial_interface.AsyncWriteFrame(&frame_b);
        break;

    case 2:
        frame_c.SetData(str_data);
        serial_interface.AsyncWriteFrame(&frame_c);
        break;

    case 3:
        frame_d.SetData(str_data);
        serial_interface.AsyncWriteFrame(&frame_d);
        break;
    }
}

/*
void SingletonTest::handleSignalExample(const QVariant& object) {
    //start button will pass in entire mainPage object

//    qDebug() << "Signal Passes In - " << object;
//    QQuickItem *item  = qobject_cast<QQuickItem*>(object.value<QObject*>());
//    qDebug() << item->width();

    // cast as a QObject to gain access to property read and write
    QObject *obj = object.value<QObject*>();

//    qDebug() << QQmlProperty::read(obj, "angle");
//    QQmlProperty::write(obj,"angle", 124);

    qDebug() << QQmlProperty::read(obj, "searchChunkCoords");
//    QJSValue* array = QQmlProperty::read(obj, "searchChunkCoords").value<QJSValue*>();


}
*/
