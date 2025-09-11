#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , useMetricUnits(true)
{
    ui->setupUi(this);

    // Connect signals and slots
    connect(ui->calculateButton, &QPushButton::clicked, this, &MainWindow::calculateTrajectory);
    connect(ui->exportButton, &QPushButton::clicked, this, &MainWindow::exportToCSV);
    connect(ui->unitComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onUnitChanged);

    updateUnitLabels();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete ballistics;
}

void MainWindow::onUnitChanged(int index) {
    useMetricUnits = (index == 0);
    updateUnitLabels();
}

void MainWindow::updateUnitLabels() {
    if (useMetricUnits) {
        ui->massEdit->setPlaceholderText("kg");
        ui->diameterEdit->setPlaceholderText("m");
        ui->muzzleVelocityEdit->setPlaceholderText("m/s");
        ui->windSpeedEdit->setPlaceholderText("m/s");
    } else {
        ui->massEdit->setPlaceholderText("lb");
        ui->diameterEdit->setPlaceholderText("in");
        ui->muzzleVelocityEdit->setPlaceholderText("ft/s");
        ui->windSpeedEdit->setPlaceholderText("ft/s");
    }
}

double MainWindow::convertToMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value * 0.453592; // lb to kg
    if (unitType == "length") return value * 0.0254; // in to m
    if (unitType == "velocity") return value * 0.3048; // ft/s to m/s
    return value;
}

double MainWindow::convertFromMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value / 0.453592; // kg to lb
    if (unitType == "length") return value / 0.0254; // m to in
    if (unitType == "velocity") return value / 0.3048; // m/s to ft/s
    return value;
}

void MainWindow::calculateTrajectory() {
    double mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    double diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");
    double launchAngle = ui->launchAngleEdit->text().toDouble();
    double windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "velocity");
    double windDirection = ui->windDirectionEdit->text().toDouble();
    double latitude = ui->latitudeEdit->text().toDouble();
    double dragCoeff = ui->dragCoeffEdit->text().toDouble();

    delete ballistics;
    ballistics = new Ballistics6DOF(mass, diameter, dragCoeff, muzzleVelocity, launchAngle, windSpeed, windDirection, latitude);

    ballistics->clearTrajectory();
    double dt = 0.01;
    for (int i = 0; i < 1000; ++i) {
        ballistics->step(dt);
    }
    plotTrajectory(ballistics->getTrajectory());
}

void MainWindow::plotTrajectory(const std::vector<State>& trajectory) {
    QVector<double> x, y;
    for (const auto& state : trajectory) {
        x << state.x;
        y << state.y;
    }
    ui->plot->clearGraphs();
    ui->plot->addGraph();
    ui->plot->graph(0)->setData(x, y);
    ui->plot->xAxis->setLabel(useMetricUnits ? "Range (m)" : "Range (ft)");
    ui->plot->yAxis->setLabel(useMetricUnits ? "Height (m)" : "Height (ft)");
    ui->plot->rescaleAxes();
    ui->plot->replot();
}

void MainWindow::exportToCSV() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save File", "", "CSV Files (*.csv)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;

    QTextStream out(&file);
    if (useMetricUnits) {
        out << "Time (s),Range (m),Height (m)\n";
    } else {
        out << "Time (s),Range (ft),Height (ft)\n";
    }
    for (const auto& state : ballistics->getTrajectory()) {
        double range = useMetricUnits ? state.x : state.x / 0.3048;
        double height = useMetricUnits ? state.y : state.y / 0.3048;
        out << state.time << "," << range << "," << height << "\n";
    }
    file.close();
}
