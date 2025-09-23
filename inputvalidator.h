/**
 * @file inputvalidator.h
 * @brief Singleton class for managing default validation values from a JSON file.
 */

#ifndef INPUTVALIDATOR_H
#define INPUTVALIDATOR_H

#include <map>
#include <string>
#include <utility>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QDebug>

/**
 * @class InputValidator
 * @brief Singleton class that provides default minimum and maximum values for input validation.
 */
class InputValidator {
public:
    /**
     * @brief Get the singleton instance of InputValidator.
     * @return Reference to the singleton instance.
     */
    static InputValidator& getInstance() {
        static InputValidator instance;
        return instance;
    }

    /**
     * @brief Get the minimum and maximum values for a specific field.
     * @param fieldName The name of the field to get validation values for.
     * @return A pair containing the minimum and maximum values.
     */
    std::pair<double, double> getValidationRange(const std::string& fieldName) const {
        auto it = validationRanges.find(fieldName);
        if (it != validationRanges.end()) {
            return it->second;
        }
        // Return a default range if the field is not found
        return std::make_pair(0.0, std::numeric_limits<double>::max());
    }

    /**
     * @brief Get the description for a specific field.
     * @param fieldName The name of the field.
     * @return The description of the field.
     */
    std::string getFieldDescription(const std::string& fieldName) const {
        auto it = fieldDescriptions.find(fieldName);
        if (it != fieldDescriptions.end()) {
            return it->second;
        }
        return "";
    }

    /**
     * @brief Set custom validation range for a specific field.
     * @param fieldName The name of the field.
     * @param min The minimum valid value.
     * @param max The maximum valid value.
     */
    void setValidationRange(const std::string& fieldName, double min, double max) {
        validationRanges[fieldName] = std::make_pair(min, max);
    }

    /**
     * @brief Check if a value is valid for a specific field.
     * @param fieldName The name of the field.
     * @param value The value to validate.
     * @return true if the value is valid, false otherwise.
     */
    bool isValid(const std::string& fieldName, double value) const {
        auto it = validationRanges.find(fieldName);
        if (it != validationRanges.end()) {
            return value >= it->second.first && value <= it->second.second;
        }
        // If field not found, any value is considered valid
        return true;
    }

    /**
     * @brief Load validation rules from a JSON file.
     * @param filePath The path to the JSON file.
     * @return true if the file was loaded successfully, false otherwise.
     */
    bool loadFromJson(const QString& filePath) {
        QFile file(filePath);
        if (!file.open(QIODevice::ReadOnly)) {
            qWarning() << "Could not open validation rules file:" << filePath;
            return false;
        }

        QByteArray data = file.readAll();
        file.close();

        QJsonDocument doc = QJsonDocument::fromJson(data);
        if (doc.isNull() || !doc.isObject()) {
            qWarning() << "Invalid JSON format in validation rules file";
            return false;
        }

        QJsonObject root = doc.object();
        if (!root.contains("validation_rules") || !root["validation_rules"].isObject()) {
            qWarning() << "Invalid JSON format: 'validation_rules' object not found";
            return false;
        }

        QJsonObject rules = root["validation_rules"].toObject();
        for (auto it = rules.begin(); it != rules.end(); ++it) {
            QString key = it.key();
            QJsonObject rule = it.value().toObject();

            if (rule.contains("min") && rule.contains("max") && rule["min"].isDouble() && rule["max"].isDouble()) {
                double min = rule["min"].toDouble();
                double max = rule["max"].toDouble();
                validationRanges[key.toStdString()] = std::make_pair(min, max);
            }

            if (rule.contains("description") && rule["description"].isString()) {
                fieldDescriptions[key.toStdString()] = rule["description"].toString().toStdString();
            }
        }

        return true;
    }

    // Delete copy constructor and assignment operator to prevent copies
    InputValidator(const InputValidator&) = delete;
    InputValidator& operator=(const InputValidator&) = delete;

private:
    // Map to store validation ranges for each field
    std::map<std::string, std::pair<double, double>> validationRanges;

    // Map to store field descriptions
    std::map<std::string, std::string> fieldDescriptions;

    /**
     * @brief Private constructor to enforce singleton pattern.
     */
    InputValidator() {
        // Try to load from JSON file first
        bool loaded = loadFromJson(":/config/input_validation_rules.json");

        // If loading from resource failed, try to load from local file
        if (!loaded) {
            loaded = loadFromJson("input_validation_rules.json");
        }

        // If still not loaded, use default values
        if (!loaded) {
            qWarning() << "Failed to load validation rules from file, using default values";

            // Initialize default validation ranges
            validationRanges["mass"] = std::make_pair(0.1, 200.0);
            fieldDescriptions["mass"] = "Mass values (grams or grains)";

            validationRanges["diameter"] = std::make_pair(0.001, 0.2);
            fieldDescriptions["diameter"] = "Diameter values (mm or inches)";

            validationRanges["muzzleVelocity"] = std::make_pair(10.0, 2000.0);
            fieldDescriptions["muzzleVelocity"] = "Muzzle velocity values (m/s or ft/s)";

            validationRanges["launchAngle"] = std::make_pair(-90.0, 90.0);
            fieldDescriptions["launchAngle"] = "Launch angle values (degrees)";

            validationRanges["windSpeed"] = std::make_pair(0.0, 100.0);
            fieldDescriptions["windSpeed"] = "Wind speed values (m/s or yd/s)";

            validationRanges["windDirection"] = std::make_pair(0.0, 360.0);
            fieldDescriptions["windDirection"] = "Wind direction values (degrees)";

            validationRanges["latitude"] = std::make_pair(-90.0, 90.0);
            fieldDescriptions["latitude"] = "Latitude values (degrees)";

            validationRanges["scopeHeight"] = std::make_pair(0.001, 0.5);
            fieldDescriptions["scopeHeight"] = "Scope height values (mm or inches)";

            validationRanges["dragCoefficient"] = std::make_pair(0.01, 5.0);
            fieldDescriptions["dragCoefficient"] = "Drag coefficient values";

            validationRanges["tableMaxRange"] = std::make_pair(1.0, 5000.0);
            fieldDescriptions["tableMaxRange"] = "Table max range values (m or yd)";

            validationRanges["tableInterval"] = std::make_pair(0.1, 1000.0);
            fieldDescriptions["tableInterval"] = "Table interval values (m or yd)";

            validationRanges["zeroRange"] = std::make_pair(1.0, 5000.0);
            fieldDescriptions["zeroRange"] = "Zero range values (m or yd)";
        }
    }
};

#endif // INPUTVALIDATOR_H
