#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "ui_mainwindow.h"
#include "bulletdata.h"
#include "inputvalidator.h"

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
    void onDropUnitChanged(int index);
    void onExitButtonClicked();
    void onActionExitTriggered();
    void validateInputsAndUpdateCalculateButton();
    void onInputFieldChanged();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    Ui::MainWindow *ui;
    BallisticsModel *ballisticsModel;
    bool useMetricUnits;
    BallisticsAlgorithm currentAlgorithm;
    std::vector<Bullet> bulletDatabase;
    double scopeHeight;  // Height of the scope above the barrel in meters
    bool hasUnsavedChanges;

private:
    enum class DropUnit {
        Inches,
        MOA,
        MIL
    };
    DropUnit dropUnit;

private:
    void plotTrajectory(const std::vector<std::array<double, 3>>& trajectory);
    double convertToMetric(double value, const QString &unitType);
    double convertFromMetric(double value, const QString &unitType);
    void updateUnitLabels();
    void updateZeroAngleLabels();
    void saveProfileToJson(const QString &fileName);
    void loadProfileFromJson(const QString &fileName);
    void populateBulletComboBox();
    double getDragCoefficient(const Bullet &bullet, double velocity, const QString &model);
    void calculateAndDisplayZeroAngle(double range);
    double getDragCoefficientAtVelocity(const Bullet &bullet, double velocity, const QString &model);
    void populateTrajectoryTable(double maxRange, double interval);
    double convertDropToDisplayUnit(double drop, double range) const;
    double moaToMeters(double moa, double range) const;
    double milToMeters(double mil, double range) const;
    bool validateInput(QLineEdit* field, double& value, const std::string& fieldName, bool allowEmpty = true);
    bool validateAllInputs();
    void setupInputValidators();
    void setupTooltips();
};
#endif // MAINWINDOW_H
