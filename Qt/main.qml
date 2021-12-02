import QtQuick 2.12
import QtQuick.Window 2.12
import TableModel 0.1

Window {
    visible: true
    width: 20 * 25
    height: 20 * 25
    title: qsTr("Path Finder")

    TableView {
        anchors.fill: parent
        columnSpacing: 1
        rowSpacing: 1
        clip: true

        model: TableModel {}

        delegate: Rectangle {
            implicitWidth: 20
            implicitHeight: 20
            border.color: black
            border.width: 1       
            color: cellColor
        }
    }
}
