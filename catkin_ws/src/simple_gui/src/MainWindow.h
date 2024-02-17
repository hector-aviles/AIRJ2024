#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QGraphicsPixmapItem>
#include <QKeyEvent>
#include <QPainter>
#include <qwt_dial_needle.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>
#include "QtRosNode.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QtRosNode* qtRosNode;

    void setRosNode(QtRosNode* qtRosNode);
    void closeEvent(QCloseEvent *event);
    void keyPressEvent(QKeyEvent* event);

public slots:
    void updateGraphicsReceived();

    void btn_start_clicked();
    void btn_change_left_clicked();
    void btn_cruise_clicked();
    void btn_follow_clicked();
    void btn_change_right_clicked();
    
private:
    Ui::MainWindow *ui;
    QPixmap pix_steering_wheel;
    QPixmap pix_speedometer;
};

#endif 
