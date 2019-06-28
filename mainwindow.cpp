#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    histogram = new IntensityHistogram;
    ui->verticalLayout->addWidget(histogram->getPointerToPlot());
    ui->verticalLayout->addWidget(histogram->getPointerToSlider());
    ui->verticalLayout->addWidget(histogram->getPointerToInversionButton());
    ui->verticalLayout->addWidget(histogram->getPointerToClarityButton());
    ui->verticalLayout->addWidget(histogram->getPointerToBaCButton());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    histogram->setDataPlotFromFile();
}

void MainWindow::on_pushButton_2_clicked()
{
    histogram->setXYValues();
    QVector<int> x = histogram->getXValues();
    QVector<double> y = histogram->getYValues();
    for (int i = 0; i < x.size(); i++)
    {
        qDebug() << "x =" << i << "y =" << y.at(i);
    }
}

void MainWindow::on_pushButton_3_clicked()
{
    histogram->clarityIsInvisible();
}

void MainWindow::on_pushButton_4_clicked()
{
    histogram->clearPlot();
}
