#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "dof3.h"
#include "dof6.h"
#include "modifiedpointmass.h"
#include "rungekutta.h"
#include "siacci.h"
#include "pejsa.h"
#include "modifiedeuler.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , useMetricUnits(true)
{
    ui->setupUi(this);

    ballisticsModel = nullptr;

    // Populate algorithm combo box
    ui->algorithmComboBox->addItem("3DOF", QVariant::fromValue(BallisticsAlgorithm::DOF3));
    ui->algorithmComboBox->addItem("6DOF", QVariant::fromValue(BallisticsAlgorithm::DOF6));
    ui->algorithmComboBox->addItem("Modified Point Mass", QVariant::fromValue(BallisticsAlgorithm::ModifiedPointMass));
    ui->algorithmComboBox->addItem("Runge-Kutta", QVariant::fromValue(BallisticsAlgorithm::RungeKutta));
    ui->algorithmComboBox->addItem("Modified Euler", QVariant::fromValue(BallisticsAlgorithm::ModifiedEuler));
    ui->algorithmComboBox->addItem("Siacci", QVariant::fromValue(BallisticsAlgorithm::Siacci));
    ui->algorithmComboBox->addItem("Pejsa", QVariant::fromValue(BallisticsAlgorithm::Pejsa));

    // Connect signals and slots
    connect(ui->calculateButton, &QPushButton::clicked, this, &MainWindow::calculateTrajectory);
    connect(ui->exportButton, &QPushButton::clicked, this, &MainWindow::exportToCSV);
    connect(ui->unitComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onUnitChanged);
    connect(ui->saveProfileButton, &QPushButton::clicked, this, &MainWindow::saveProfile);
    connect(ui->loadProfileButton, &QPushButton::clicked, this, &MainWindow::loadProfile);
    connect(ui->algorithmComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onAlgorithmChanged);
    connect(ui->bulletComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onBulletSelected);
    connect(ui->loadBulletDataButton, &QPushButton::clicked, this, &MainWindow::loadBulletDataFile);
    connect(ui->calculateZeroButton, &QPushButton::clicked, this, &MainWindow::calculateZeroAngle);
    connect(ui->generateTableButton, &QPushButton::clicked, this, &MainWindow::generateTrajectoryTable);

    updateUnitLabels();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::generateTrajectoryTable() {
    bool ok;
    double maxRange = ui->tableMaxRangeEdit->text().toDouble(&ok);
    if (!ok || maxRange <= 0) {
        QMessageBox::warning(this, "Error", "Please enter a valid maximum range.");
        return;
    }

    double interval = ui->tableIntervalEdit->text().toDouble(&ok);
    if (!ok || interval <= 0) {
        QMessageBox::warning(this, "Error", "Please enter a valid interval.");
        return;
    }

    // Convert to meters if in imperial mode
    if (!useMetricUnits) {
        maxRange *= 0.9144; // yards to meters
        interval *= 0.9144; // yards to meters
    }

    populateTrajectoryTable(maxRange, interval);

    // Switch to the Trajectory Table tab
    ui->tabWidget->setCurrentIndex(3); // Assuming this is the 4th tab (index 3)
}


void MainWindow::calculateZeroAngle() {
    bool ok;
    double range = ui->zeroRangeEdit->text().toDouble(&ok);
    if (!ok || range <= 0) {
        QMessageBox::warning(this, "Error", "Please enter a valid range.");
        return;
    }

    // Convert range to meters if in imperial mode
    if (!useMetricUnits) {
        range *= 0.9144; // yards to meters
    }

    calculateAndDisplayZeroAngle(range);

    // Switch to the Input Parameters tab after calculation
    ui->tabWidget->setCurrentIndex(0);
}

void MainWindow::populateTrajectoryTable(double maxRange, double interval) {
    int bulletIndex = ui->bulletComboBox->currentIndex();
    if (bulletIndex < 0 || static_cast<size_t>(bulletIndex) >= bulletDatabase.size()) {
        QMessageBox::warning(this, "Error", "Please select a bullet profile.");
        return;
    }

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(bulletIndex)];
    double mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    double diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");
    double launchAngle = ui->launchAngleEdit->text().toDouble();
    double windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "windSpeed");
    double windDirection = ui->windDirectionEdit->text().toDouble();
    double latitude = ui->latitudeEdit->text().toDouble();

    // Get drag coefficient based on selected model
    double dragCoeff = getDragCoefficient(selectedBullet, muzzleVelocity, "G7");

    // Clear and set up the table
    ui->trajectoryTable->clear();
    ui->trajectoryTable->setRowCount(0);
    ui->trajectoryTable->setColumnCount(9);

    // Set table headers
    QStringList headers;
    if (useMetricUnits) {
        headers << "Range (m)" << "Height (m)" << "Time (s)" << "Velocity (m/s)"
                << "Energy (J)" << "Drop (m)" << "Windage (m)"
                << "Vertical Velocity (m/s)" << "Horizontal Velocity (m/s)";
    } else {
        headers << "Range (yd)" << "Height (yd)" << "Time (s)" << "Velocity (ft/s)"
                << "Energy (ft-lb)" << "Drop (in)" << "Windage (in)"
                << "Vertical Velocity (ft/s)" << "Horizontal Velocity (ft/s)";
    }
    ui->trajectoryTable->setHorizontalHeaderLabels(headers);

    // Create a temporary ballistics model for table generation
    BallisticsModel* tableModel = new DOF6();
    tableModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, launchAngle, windSpeed, windDirection, latitude);

    // Generate trajectory data
    double dt = 0.01;
    std::vector<std::array<double, 3>> fullTrajectory;
    for (int i = 0; i < 10000; ++i) {  // Increased iterations for longer ranges
        tableModel->step(dt);
        const auto& trajectory = tableModel->getTrajectory();
        if (!trajectory.empty()) {
            fullTrajectory = trajectory;
        }
        // Stop if the bullet has hit the ground (y < 0)
        if (!trajectory.empty() && trajectory.back()[1] < 0) {
            break;
        }
    }

    // Find the maximum height for drop calculation
    double maxHeight = 0;
    for (size_t i = 0; i < fullTrajectory.size(); ++i) {
        if (fullTrajectory[i][1] > maxHeight) {
            maxHeight = fullTrajectory[i][1];
        }
    }

    // Add rows to the table at specified intervals
    int row = 0;
    for (double range = 0; range <= maxRange; range += interval) {
        ui->trajectoryTable->insertRow(row);

        // Find the trajectory point closest to the current range
        double closestX = 0, closestY = 0, closestT = 0;
        size_t closestIndex = 0;
        double minDist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < fullTrajectory.size(); ++i) {
            double dist = std::abs(fullTrajectory[i][0] - range);
            if (dist < minDist) {
                minDist = dist;
                closestX = fullTrajectory[i][0];
                closestY = fullTrajectory[i][1];
                closestT = fullTrajectory[i][2];
                closestIndex = i;
            }
        }

        // Calculate velocity components
        double vx = 0.0, vy = 0.0, v = 0.0;
        if (closestIndex > 0 && closestIndex < fullTrajectory.size()) {
            double dt = fullTrajectory[closestIndex][2] - fullTrajectory[closestIndex-1][2];
            if (dt > 0) {
                vx = (fullTrajectory[closestIndex][0] - fullTrajectory[closestIndex-1][0]) / dt;
                vy = (fullTrajectory[closestIndex][1] - fullTrajectory[closestIndex-1][1]) / dt;
                v = std::sqrt(vx*vx + vy*vy);
            }
        }

        // Calculate energy (E = 0.5 * m * v^2)
        double energy = 0.5 * mass * v * v;  // in Joules

        // Calculate drop (difference between max height and current height)
        double drop = maxHeight - closestY;

        // Calculate windage (lateral displacement due to wind)
        // This is a simplified calculation - actual windage would require more complex modeling
        double windage = 0.0;
        if (windSpeed > 0) {
            // Simple approximation: windage increases with range and wind speed
            windage = 0.001 * closestX * windSpeed * std::sin(windDirection);
        }

        // Convert values to display units
        double displayRange = useMetricUnits ? closestX : closestX / 0.9144;
        double displayHeight = useMetricUnits ? closestY : closestY / 0.9144;
        double displayV = useMetricUnits ? v : v / 0.3048;
        double displayVx = useMetricUnits ? vx : vx / 0.3048;
        double displayVy = useMetricUnits ? vy : vy / 0.3048;
        double displayDrop = useMetricUnits ? drop : drop / 0.0254;  // meters to inches for drop
        double displayWindage = useMetricUnits ? windage : windage / 0.0254;  // meters to inches for windage
        double displayEnergy = useMetricUnits ? energy : energy * 0.737562;  // Joules to ft-lb

        // Add items to the table
        ui->trajectoryTable->setItem(row, 0, new QTableWidgetItem(QString::number(displayRange, 'f', 1)));
        ui->trajectoryTable->setItem(row, 1, new QTableWidgetItem(QString::number(displayHeight, 'f', 2)));
        ui->trajectoryTable->setItem(row, 2, new QTableWidgetItem(QString::number(closestT, 'f', 2)));
        ui->trajectoryTable->setItem(row, 3, new QTableWidgetItem(QString::number(displayV, 'f', 1)));
        ui->trajectoryTable->setItem(row, 4, new QTableWidgetItem(QString::number(displayEnergy, 'f', 1)));
        ui->trajectoryTable->setItem(row, 5, new QTableWidgetItem(QString::number(displayDrop, 'f', 2)));
        ui->trajectoryTable->setItem(row, 6, new QTableWidgetItem(QString::number(displayWindage, 'f', 2)));
        ui->trajectoryTable->setItem(row, 7, new QTableWidgetItem(QString::number(displayVy, 'f', 1)));
        ui->trajectoryTable->setItem(row, 8, new QTableWidgetItem(QString::number(displayVx, 'f', 1)));

        row++;
    }

    // Resize columns to fit content
    ui->trajectoryTable->resizeColumnsToContents();

    delete tableModel;
}

void MainWindow::calculateAndDisplayZeroAngle(double range) {
    int bulletIndex = ui->bulletComboBox->currentIndex();
    if (bulletIndex < 0 || static_cast<size_t>(bulletIndex) >= bulletDatabase.size()) {
        QMessageBox::warning(this, "Error", "Please select a bullet profile.");
        return;
    }

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(bulletIndex)];
    double mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    double diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");
    double windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "windSpeed");
    double windDirection = ui->windDirectionEdit->text().toDouble();
    double latitude = ui->latitudeEdit->text().toDouble();

    // Get drag coefficient based on selected model
    double dragCoeff = getDragCoefficient(selectedBullet, muzzleVelocity, "G7");

    // Create a temporary ballistics model for zero angle calculation
    BallisticsModel* zeroModel = new DOF6(); // Using 6DOF for zero angle calculation
    zeroModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, 0, windSpeed, windDirection, latitude);

    // Binary search to find the zero angle
    double lowAngle = -0.1; // -0.1 radians
    double highAngle = 0.5;  // 0.5 radians (about 28.6 degrees)
    double bestAngle = 0.0;
    double minError = std::numeric_limits<double>::max();
    const double tolerance = 0.0001; // 0.0001 radians tolerance
    const int maxIterations = 100;

    for (int i = 0; i < maxIterations; ++i) {
        double midAngle1 = lowAngle + (highAngle - lowAngle) / 3.0;
        double midAngle2 = highAngle - (highAngle - lowAngle) / 3.0;

        // Test midAngle1
        zeroModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, midAngle1, windSpeed, windDirection, latitude);
        double dt = 0.01;
        double currentX = 0.0;
        double currentY = 0.0;
        for (int step = 0; step < 5000; ++step) {
            zeroModel->step(dt);
            const auto& trajectory = zeroModel->getTrajectory();
            if (!trajectory.empty()) {
                currentX = trajectory.back()[0];
                currentY = trajectory.back()[1];
            }
            if (currentX >= range) {
                break;
            }
        }

        double error1 = std::abs(currentY);

        // Test midAngle2
        zeroModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, midAngle2, windSpeed, windDirection, latitude);
        currentX = 0.0;
        currentY = 0.0;
        for (int step = 0; step < 5000; ++step) {
            zeroModel->step(dt);
            const auto& trajectory = zeroModel->getTrajectory();
            if (!trajectory.empty()) {
                currentX = trajectory.back()[0];
                currentY = trajectory.back()[1];
            }
            if (currentX >= range) {
                break;
            }
        }

        double error2 = std::abs(currentY);

        if (error1 < minError) {
            minError = error1;
            bestAngle = midAngle1;
        }
        if (error2 < minError) {
            minError = error2;
            bestAngle = midAngle2;
        }

        if (error1 < error2) {
            highAngle = midAngle2;
        } else {
            lowAngle = midAngle1;
        }

        if (std::abs(highAngle - lowAngle) < tolerance) {
            break;
        }
    }

    delete zeroModel;

    // Convert angle to degrees for display
    double zeroAngleDegrees = bestAngle * 180.0 / M_PI;

    // Display the zero angle
    ui->zeroAngleValue->setText(QString::number(zeroAngleDegrees, 'f', 2) + "°");

    // Optionally set this angle as the launch angle
    ui->launchAngleEdit->setText(QString::number(bestAngle, 'f', 4));
}

void MainWindow::loadBulletDataFile() {
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "Open Bullet Data File",
        QDir::homePath(),
        "JSON Files (*.json)"
        );

    if (filePath.isEmpty()) {
        return;
    }

    bulletDatabase = BulletData::loadBulletData(filePath);
    if (bulletDatabase.empty()) {
        QMessageBox::warning(this, "Error", "Failed to load bullet data.");
        return;
    }

    populateBulletComboBox();
    QMessageBox::information(this, "Success", "Bullet data loaded successfully.");

    // Switch to the Input Parameters tab after loading
    ui->tabWidget->setCurrentIndex(0);
}

void MainWindow::populateBulletComboBox() {
    ui->bulletComboBox->clear();
    for (const Bullet &bullet : bulletDatabase) {
        QString bulletInfo = QString("%1 %2 %3gr")
        .arg(bullet.manufacturer, bullet.model)
        .arg(bullet.weight_gr);
        ui->bulletComboBox->addItem(bulletInfo, QVariant::fromValue(bullet));
    }
}

void MainWindow::onBulletSelected(int index) {
    if (index < 0 || static_cast<size_t>(index) >= bulletDatabase.size()) return;

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(index)];
    if (useMetricUnits) {
        ui->massEdit->setText(QString::number(selectedBullet.weight_g, 'f', 2));
        ui->diameterEdit->setText(QString::number(selectedBullet.diameter_mm, 'f', 2));
    } else {
        ui->massEdit->setText(QString::number(selectedBullet.weight_gr, 'f', 1));
        ui->diameterEdit->setText(QString::number(selectedBullet.diameter_in, 'f', 3));
    }
}

double MainWindow::getDragCoefficient(const Bullet &bullet, double velocity, const QString &model) {
    return getDragCoefficientAtVelocity(bullet, velocity, model);
}

void MainWindow::onAlgorithmChanged(int index) {
    currentAlgorithm = ui->algorithmComboBox->itemData(index).value<BallisticsAlgorithm>();
}

void MainWindow::onUnitChanged(int index) {
    useMetricUnits = (index == 0);
    updateUnitLabels();
}

void MainWindow::updateUnitLabels() {
    if (useMetricUnits) {
        ui->massLabel->setText("Mass (g):");
        ui->diameterLabel->setText("Diameter (mm):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (m/s):");
        ui->windSpeedLabel->setText("Wind Speed (m/s):");
        ui->plot->xAxis->setLabel("Range (m)");
        ui->plot->yAxis->setLabel("Height (m)");
        ui->zeroRangeLabel->setText("Zero Range (m):");
        ui->tableMaxRangeLabel->setText("Max Range (m):");
        ui->tableIntervalLabel->setText("Interval (m):");
    } else {
        ui->massLabel->setText("Mass (gr):");
        ui->diameterLabel->setText("Diameter (in):");
        ui->muzzleVelocityLabel->setText("Muzzle Velocity (ft/s):");
        ui->windSpeedLabel->setText("Wind Speed (yd/s):");
        ui->plot->xAxis->setLabel("Range (yd)");
        ui->plot->yAxis->setLabel("Height (yd)");
        ui->zeroRangeLabel->setText("Zero Range (yd):");
        ui->tableMaxRangeLabel->setText("Max Range (yd):");
        ui->tableIntervalLabel->setText("Interval (yd):");    }

    // Refresh the plot to show the updated labels
    ui->plot->replot();}

double MainWindow::convertToMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value / 15.4324; // grains to grams
    if (unitType == "length") return value * 0.0254; // inches to meters
    if (unitType == "range") return value * 0.9144; // yards to meters
    if (unitType == "velocity") return value * 0.3048; // ft/s to m/s
    if (unitType == "windSpeed") return value * 0.9144; // yd/s to m/s
    return value;
}

double MainWindow::convertFromMetric(double value, const QString &unitType) {
    if (useMetricUnits) return value;
    if (unitType == "mass") return value * 15.4324; // grams to grains
    if (unitType == "length") return value / 0.0254; // meters to inches
    if (unitType == "range") return value / 0.9144; // meters to yards
    if (unitType == "velocity") return value / 0.3048; // m/s to ft/s
    if (unitType == "windSpeed") return value / 0.9144; // m/s to yd/s
    return value;
}

double MainWindow::getDragCoefficientAtVelocity(const Bullet &bullet, double velocity, const QString &model)
{
    QVariantMap dragData = bullet.drag_coefficients.value(model).toMap();
    if (dragData.isEmpty()) return 0.5; // Default value

    // If we have a constant BC, use it
    if (dragData.contains("bc")) {
        double bc = dragData["bc"].toDouble();
        double diameter = bullet.diameter_mm / 1000.0; // Convert to meters
        double mass = bullet.weight_g / 1000.0; // Convert to kg
        return (mass / (diameter * diameter)) / bc;
    }

    // If we have speed-dependent coefficients, find the closest
    QVariantList supersonicData = dragData["supersonic"].toList();
    if (supersonicData.isEmpty()) return 0.5; // Default value

    // Convert velocity to ft/s for comparison with the table
    double velocityFtPerSec = velocity / 0.3048;

    // Find the closest velocity range
    double closestCd = supersonicData[0].toMap()["cd"].toDouble();
    double minDiff = std::abs(supersonicData[0].toMap()["velocity"].toDouble() - velocityFtPerSec);

    for (const QVariant &entry : supersonicData) {
        QVariantMap data = entry.toMap();
        double entryVelocity = data["velocity"].toDouble();
        double diff = std::abs(entryVelocity - velocityFtPerSec);
        if (diff < minDiff) {
            minDiff = diff;
            closestCd = data["cd"].toDouble();
        }
    }

    return closestCd;
}

void MainWindow::calculateTrajectory() {
    int bulletIndex = ui->bulletComboBox->currentIndex();
    if (bulletIndex < 0 || static_cast<size_t>(bulletIndex) >= bulletDatabase.size()) {
        QMessageBox::warning(this, "Error", "Please select a bullet profile.");
        return;
    }

    Bullet selectedBullet = bulletDatabase[static_cast<size_t>(bulletIndex)];
    double mass = convertToMetric(ui->massEdit->text().toDouble(), "mass");
    double diameter = convertToMetric(ui->diameterEdit->text().toDouble(), "length");
    double muzzleVelocity = convertToMetric(ui->muzzleVelocityEdit->text().toDouble(), "velocity");
    double launchAngle = ui->launchAngleEdit->text().toDouble();
// possible edit
//  double launchAngle = ui->launchAngleEdit->text().toDouble()
    double windSpeed = convertToMetric(ui->windSpeedEdit->text().toDouble(), "windSpeed");
    double windDirection = ui->windDirectionEdit->text().toDouble();
    double latitude = ui->latitudeEdit->text().toDouble();

    // Get drag coefficient based on selected model
    double dragCoeff = getDragCoefficient(selectedBullet, muzzleVelocity, "G7");

    delete ballisticsModel;
    switch (currentAlgorithm) {
    case BallisticsAlgorithm::DOF3:
        ballisticsModel = new DOF3();
        break;
    case BallisticsAlgorithm::DOF6:
        ballisticsModel = new DOF6();
        break;
    case BallisticsAlgorithm::ModifiedPointMass:
        ballisticsModel = new ModifiedPointMass();
        break;
    case BallisticsAlgorithm::RungeKutta:
        ballisticsModel = new RungeKutta();
        break;
    case BallisticsAlgorithm::ModifiedEuler:
        ballisticsModel = new ModifiedEuler();
        break;
    case BallisticsAlgorithm::Siacci:
        ballisticsModel = new Siacci();
        break;
    case BallisticsAlgorithm::Pejsa:
        ballisticsModel = new Pejsa();
        break;
    }

    ballisticsModel->setParameters(mass, diameter, dragCoeff, muzzleVelocity, launchAngle, windSpeed, windDirection, latitude);

    double dt = 0.01;
    for (int i = 0; i < 1000; ++i) {
        ballisticsModel->step(dt);
    }

    plotTrajectory(ballisticsModel->getTrajectory());

    // Switch to the Trajectory Visualization tab after calculation
    ui->tabWidget->setCurrentIndex(1);
}

void MainWindow::plotTrajectory(const std::vector<std::array<double, 3>>& trajectory) {
    QVector<double> x, y;
    for (const auto& point : trajectory) {
        double displayX = useMetricUnits ? point[0] : point[0] / 0.9144; // meters to yards
        double displayY = useMetricUnits ? point[1] : point[1] / 0.9144; // meters to yards
        x << displayX;
        y << displayY;
    }

    ui->plot->clearGraphs();
    ui->plot->addGraph();
    ui->plot->graph(0)->setData(x, y);

    // Set axis labels based on unit system
    if (useMetricUnits) {
        ui->plot->xAxis->setLabel("Range (m)");
        ui->plot->yAxis->setLabel("Height (m)");
    } else {
        ui->plot->xAxis->setLabel("Range (yd)");
        ui->plot->yAxis->setLabel("Height (yd)");
    }

    ui->plot->rescaleAxes();
    ui->plot->replot();
}

void MainWindow::exportToCSV() {
    QString fileName = QFileDialog::getSaveFileName(this, "Save Trajectory Data", "", "CSV Files (*.csv)");
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "Error", "Could not open file for writing.");
        return;
    }

    QTextStream out(&file);

    // Write header with metadata
    out << "# Ballistic Trajectory Data\n";
    out << "# Algorithm: ";
    switch (currentAlgorithm) {
    case BallisticsAlgorithm::DOF3: out << "3DOF\n"; break;
    case BallisticsAlgorithm::DOF6: out << "6DOF\n"; break;
    case BallisticsAlgorithm::ModifiedPointMass: out << "Modified Point Mass\n"; break;
    case BallisticsAlgorithm::RungeKutta: out << "Runge-Kutta\n"; break;
    case BallisticsAlgorithm::ModifiedEuler: out << "Modified Euler\n"; break;
    case BallisticsAlgorithm::Siacci: out << "Siacci\n"; break;
    case BallisticsAlgorithm::Pejsa: out << "Pejsa\n"; break;
    }
    out << "# Unit System: " << (useMetricUnits ? "Metric" : "Imperial") << "\n";
    out << "# Parameters:\n";
    out << "# Mass: " << (useMetricUnits ? ui->massEdit->text() + " g" : ui->massEdit->text() + " gr") << "\n";
    out << "# Diameter: " << (useMetricUnits ? ui->diameterEdit->text() + " mm" : ui->diameterEdit->text() + " in") << "\n";
    out << "# Muzzle Velocity: " << (useMetricUnits ? ui->muzzleVelocityEdit->text() + " m/s" : ui->muzzleVelocityEdit->text() + " ft/s") << "\n";
    out << "# Launch Angle: " << ui->launchAngleEdit->text() << " rad\n";
    out << "# Wind Speed: " << (useMetricUnits ? ui->windSpeedEdit->text() + " m/s" : ui->windSpeedEdit->text() + " yd/s") << "\n";
    out << "# Wind Direction: " << ui->windDirectionEdit->text() << " rad\n";
    out << "# Latitude: " << ui->latitudeEdit->text() << " rad\n";
    out << "# Drag Coefficient: " << ui->dragCoeffEdit->text() << "\n";
    out << "#\n";

    // Write CSV header
    if (useMetricUnits) {
        out << "Time (s),Range (m),Height (m),Velocity (m/s),Velocity X (m/s),Velocity Y (m/s)\n";
    } else {
        out << "Time (s),Range (yd),Height (yd),Velocity (ft/s),Velocity X (ft/s),Velocity Y (ft/s)\n";
    }

    // Get trajectory data
    const auto& trajectory = ballisticsModel->getTrajectory();

    // Write trajectory data
    for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto& point = trajectory[i];
        double x = point[0];
        double y = point[1];
        double t = point[2];

        // Convert to imperial if needed
        double displayX = useMetricUnits ? x : x / 0.9144; // meters to yards
        double displayY = useMetricUnits ? y : y / 0.9144; // meters to yards

        // For velocity, calculate from trajectory points
        double vx = 0.0, vy = 0.0, v = 0.0;
        if (i < trajectory.size() - 1) {
            double dx = trajectory[i+1][0] - x;
            double dy = trajectory[i+1][1] - y;
            double dt = trajectory[i+1][2] - t;
            vx = dx / dt;
            vy = dy / dt;
            v = sqrt(vx * vx + vy * vy);
        }

        double displayVx = useMetricUnits ? vx : vx / 0.3048; // m/s to ft/s
        double displayVy = useMetricUnits ? vy : vy / 0.3048; // m/s to ft/s
        double displayV = useMetricUnits ? v : v / 0.3048; // m/s to ft/s

        out << t << "," << displayX << "," << displayY << ","
            << displayV << "," << displayVx << "," << displayVy << "\n";
    }

    file.close();
    QMessageBox::information(this, "Success", "Trajectory data exported successfully.");
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
