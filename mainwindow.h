#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "ui_mainwindow.h"
#include "bulletdata.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

enum class BallisticsAlgorithm {
    DOF3,
    DOF6,
    ModifiedPointMass,
    RungeKutta,
    ModifiedEuler,
    Siacci,
    Pejsa
};

class BallisticsModel; // Forward declaration

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
    void onAlgorithmChanged(int index);
    void onBulletSelected(int index);
    void loadBulletDataFile();
    void calculateZeroAngle();
    void generateTrajectoryTable();

private:
    Ui::MainWindow *ui;
    BallisticsModel *ballisticsModel;
    bool useMetricUnits;
    BallisticsAlgorithm currentAlgorithm;
    std::vector<Bullet> bulletDatabase;
    double scopeHeight;  // Height of the scope above the barrel in meters
private:
    void plotTrajectory(const std::vector<std::array<double, 3>>& trajectory);
    double convertToMetric(double value, const QString &unitType);
    double convertFromMetric(double value, const QString &unitType);
    void updateUnitLabels();
    void saveProfileToJson(const QString &fileName);
    void loadProfileFromJson(const QString &fileName);
    void populateBulletComboBox();
    double getDragCoefficient(const Bullet &bullet, double velocity, const QString &model);
    void calculateAndDisplayZeroAngle(double range);
    double getDragCoefficientAtVelocity(const Bullet &bullet, double velocity, const QString &model);
    void populateTrajectoryTable(double maxRange, double interval);
};
#endif // MAINWINDOW_H
