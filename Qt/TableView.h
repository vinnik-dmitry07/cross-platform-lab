#pragma once

#include <QObject>
#include <QAbstractTableModel>

class TableView : public QAbstractTableModel
{
	Q_OBJECT

public:
	TableView(QObject *parent);
	~TableView();
};
