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
    connect(ui->saveProfileButton, &QPushButton::clicked, this, &MainWindow::saveProfile);
    connect(ui->loadProfileButton, &QPushButton::clicked, this, &MainWindow::loadProfile);

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
        ui->massLabel->setText("Mass (kg):");
        ui->diameterLabel->setText("Diameter (m):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (m/s):");
        ui->windSpeedLabel->setText("Wind Speed (m/s):");
        ui->plot->xAxis->setLabel("Range (m)");
        ui->plot->yAxis->setLabel("Height (m)");
    } else {
        ui->massLabel->setText("Mass (lb):");
        ui->diameterLabel->setText("Diameter (in):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (ft/s):");
        ui->windSpeedLabel->setText("Wind Speed (ft/s):");
        ui->plot->xAxis->setLabel("Range (ft)");
        ui->plot->yAxis->setLabel("Height (ft)");
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

void MainWindow::saveProfile() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save Profile", "", "JSON Files (*.json)");
    if (fileName.isEmpty()) return;
    saveProfileToJson(fileName);
}

void MainWindow::loadProfile() {
    QString fileName = QFileDialog::getOpenFileName(this, "Load Profile", "", "JSON Files (*.json)");
    if (fileName.isEmpty()) return;
    loadProfileFromJson(fileName);
}

void MainWindow::saveProfileToJson(const QString &fileName) {
    QJsonObject profile;
    profile["mass"] = ui->massEdit->text();
    profile["diameter"] = ui->diameterEdit->text();
    profile["dragCoeff"] = ui->dragCoeffEdit->text();
    profile["muzzleVelocity"] = ui->muzzleVelocityEdit->text();
    profile["launchAngle"] = ui->launchAngleEdit->text();
    profile["windSpeed"] = ui->windSpeedEdit->text();
    profile["windDirection"] = ui->windDirectionEdit->text();
    profile["latitude"] = ui->latitudeEdit->text();
    profile["unitSystem"] = ui->unitComboBox->currentIndex();

    QJsonDocument doc(profile);
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, "Error", "Could not save profile.");
        return;
    }
    file.write(doc.toJson());
    file.close();
}

void MainWindow::loadProfileFromJson(const QString &fileName) {
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(this, "Error", "Could not load profile.");
        return;
    }
    QByteArray data = file.readAll();
    file.close();

    QJsonDocument doc(QJsonDocument::fromJson(data));
    QJsonObject profile = doc.object();

    ui->massEdit->setText(profile["mass"].toString());
    ui->diameterEdit->setText(profile["diameter"].toString());
    ui->dragCoeffEdit->setText(profile["dragCoeff"].toString());
    ui->muzzleVelocityEdit->setText(profile["muzzleVelocity"].toString());
    ui->launchAngleEdit->setText(profile["launchAngle"].toString());
    ui->windSpeedEdit->setText(profile["windSpeed"].toString());
    ui->windDirectionEdit->setText(profile["windDirection"].toString());
    ui->latitudeEdit->setText(profile["latitude"].toString());
    ui->unitComboBox->setCurrentIndex(profile["unitSystem"].toInt());

    onUnitChanged(ui->unitComboBox->currentIndex());
}
