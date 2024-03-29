#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "intensityhistogram.h"
#include <QMouseEvent>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

private:
    Ui::MainWindow *ui;
    IntensityHistogram *histogram;

};

#endif // MAINWINDOW_H
