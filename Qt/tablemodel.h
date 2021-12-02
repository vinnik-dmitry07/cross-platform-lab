#pragma once

#include <QObject>
#include <QAbstractTableModel>
#include <random>
#include <string>
#include "utils.h"
#include "a_star.h"


class TableModel : public QAbstractTableModel
{
	Q_OBJECT
public:
    int n = 25;
    const QVariant color_map[4] = { "white", "black", "green", "green" };

    TableModel::TableModel(QObject* parent = nullptr) : QAbstractTableModel(parent), grid(n, std::vector<int>(n, 0))
    {
        MakeGrid(grid);

        std::random_device rd;   // obtain a random number from hardware
        std::mt19937 eng(rd());  // seed the generator
        std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

        Node start(distr(eng), distr(eng), 0, 0, 0, 0);
        Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

        start.id_ = start.x_ * n + start.y_;
        start.pid_ = start.x_ * n + start.y_;
        goal.id_ = goal.x_ * n + goal.y_;
        start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
        // Make sure start and goal are not obstacles and their ids are correctly
        // assigned.
        grid[start.x_][start.y_] = 0;
        grid[goal.x_][goal.y_] = 0;

        AStar a_star(grid);
        {
            const auto [path_found, path_vector] = a_star.Plan(start, goal);
            FillPath(path_vector, start, goal, grid);
        }
    }

    int rowCount(const QModelIndex & = QModelIndex()) const override
    {
        return n;
    }

    int columnCount(const QModelIndex & = QModelIndex()) const override
    {
        return n;
    }

    QVariant data(const QModelIndex& index, int role) const override
    {
        switch (role) {
        case Qt::DisplayRole:
            return color_map[grid[index.row()][index.column()]];
        default:
            break;
        }

        return QVariant();
    }

    QHash<int, QByteArray> roleNames() const override
    {
        return { {Qt::DisplayRole, "cellColor"} };
    }
private:
    std::vector<std::vector<int>> grid;
};
