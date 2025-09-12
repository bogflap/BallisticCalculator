#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "ui_mainwindow.h"
#include "ballistics.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void calculateTrajectory();
    void exportToCSV();
    void onUnitChanged(int index);
    void saveProfile();
    void loadProfile();

private:
    Ui::MainWindow *ui;
    Ballistics6DOF *ballistics;
    bool useMetricUnits;

private:
    void plotTrajectory(const std::vector<State>& trajectory);
    double convertToMetric(double value, const QString &unitType);
    double convertFromMetric(double value, const QString &unitType);
    void updateUnitLabels();
    void saveProfileToJson(const QString &fileName);
    void loadProfileFromJson(const QString &fileName);
};
#endif // MAINWINDOW_H
