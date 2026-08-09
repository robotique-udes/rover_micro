#ifndef WIFI_SCIENCE_LOGGER_HPP
#define WIFI_SCIENCE_LOGGER_HPP

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

class WiFiScienceLogger
{
    static constexpr uint16_t SERVER_PORT = 80;
    static constexpr const char* CSV_FILE_PATH = "/science_data.csv";

  public:
    struct SensorSample
    {
        uint32_t timestampMs;
        uint32_t sampleIndex;
        int32_t sensor1;
        int32_t sensor2;
        int32_t sensor3;
        uint16_t humidity;
    };

    WiFiScienceLogger(const char* apSsid_ = "Rover-Science-ESP32", const char* apPass_ = "rover1234"):
        _apSsid(apSsid_),
        _apPass(apPass_),
        _server(SERVER_PORT)
    {
    }

    void init(bool enableAP = true)
    {
        if (!LittleFS.begin(true))
        {
            Serial.println("[WiFiLogger] LittleFS mount failed, formatting...");
        }

        if (!LittleFS.exists(CSV_FILE_PATH))
        {
            this->createEmptyCsvFile();
        }
        else
        {
            File f = LittleFS.open(CSV_FILE_PATH, "r");
            if (f)
            {
                _fileSizeBytes = f.size();
                f.close();
            }
        }

        if (enableAP)
        {
            WiFi.mode(WIFI_AP);
            WiFi.softAP(_apSsid, _apPass);
            _apIP = WiFi.softAPIP();
        }

        _server.on("/", [this]() { this->handleRoot(); });
        _server.on("/data.csv", [this]() { this->handleCsvDownload(); });
        _server.on("/api/data", [this]() { this->handleJsonData(); });
        _server.on("/clear", [this]() { this->handleClear(); });

        _server.begin();
    }

    void logSample(uint32_t sampleIndex_, int32_t s1_, int32_t s2_, int32_t s3_, uint16_t humidity_ = 0)
    {
        _latestSample.timestampMs = millis();
        _latestSample.sampleIndex = sampleIndex_;
        _latestSample.sensor1 = s1_;
        _latestSample.sensor2 = s2_;
        _latestSample.sensor3 = s3_;
        _latestSample.humidity = humidity_;

        File f = LittleFS.open(CSV_FILE_PATH, "a");
        if (f)
        {
            f.print(_latestSample.timestampMs);
            f.print(',');
            f.print(_latestSample.sampleIndex);
            f.print(',');
            f.print(_latestSample.sensor1);
            f.print(',');
            f.print(_latestSample.sensor2);
            f.print(',');
            f.print(_latestSample.sensor3);
            f.print(',');
            f.println(_latestSample.humidity);
            _fileSizeBytes = f.size();
            f.close();
            _sampleCount++;
        }
    }

    void update()
    {
        _server.handleClient();
    }

    IPAddress getIP() const
    {
        return _apIP;
    }

    size_t getSampleCount() const
    {
        return _sampleCount;
    }

  private:
    void createEmptyCsvFile()
    {
        File f = LittleFS.open(CSV_FILE_PATH, "w");
        if (f)
        {
            f.println("timestamp_ms,sample_index,sensor_1_ppm,sensor_2_ppm,sensor_3_ppm,humidity");
            _fileSizeBytes = f.size();
            f.close();
            _sampleCount = 0;
        }
    }

    void handleRoot()
    {
        String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
        html += "<title>Rover Science Sensors</title>";
        html += "<meta http-equiv='refresh' content='2'>";
        html += "<style>";
        html += "body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #121214; color: #E1E1E6; margin: 0; padding: 20px; display: flex; justify-content: center; }";
        html += ".container { max-width: 850px; width: 100%; }";
        html += "h1 { color: #00B4D8; text-align: center; margin-bottom: 25px; }";
        html += ".cards { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 15px; margin-bottom: 25px; }";
        html += ".card { background: #202024; border-radius: 12px; padding: 20px; text-align: center; border: 1px solid #323238; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }";
        html += ".card h3 { color: #A8A8B3; margin-top: 0; font-size: 1rem; }";
        html += ".value { font-size: 2.2rem; font-weight: bold; color: #00E676; margin: 10px 0; }";
        html += ".unit { font-size: 0.9rem; color: #7C7C8A; }";
        html += ".btn-group { display: flex; gap: 15px; justify-content: center; margin-top: 20px; }";
        html += ".btn { padding: 12px 24px; border-radius: 8px; font-weight: bold; text-decoration: none; cursor: pointer; border: none; font-size: 1rem; transition: background 0.2s; }";
        html += ".btn-primary { background: #00B4D8; color: #121214; }";
        html += ".btn-primary:hover { background: #90E0EF; }";
        html += ".btn-danger { background: #E53935; color: #FFF; }";
        html += ".btn-danger:hover { background: #EF5350; }";
        html += ".stats { color: #7C7C8A; text-align: center; margin-top: 20px; font-size: 0.9rem; line-height: 1.6; }";
        html += "</style></head><body><div class='container'>";
        html += "<h1>🔬 Rover Science Module Dashboard</h1>";
        html += "<div class='cards'>";

        html += "<div class='card'><h3>CO₂ Sensor 1 (0x68)</h3><div class='value'>" + String(_latestSample.sensor1 >= 0 ? String(_latestSample.sensor1) : "N/A") + "</div><div class='unit'>PPM</div></div>";
        html += "<div class='card'><h3>CO₂ Sensor 2 (0x69)</h3><div class='value'>" + String(_latestSample.sensor2 >= 0 ? String(_latestSample.sensor2) : "N/A") + "</div><div class='unit'>PPM</div></div>";
        html += "<div class='card'><h3>CO₂ Sensor 3 (0x70)</h3><div class='value'>" + String(_latestSample.sensor3 >= 0 ? String(_latestSample.sensor3) : "N/A") + "</div><div class='unit'>PPM</div></div>";

        html += "</div>";
        html += "<div class='btn-group'>";
        html += "<a href='/data.csv' class='btn btn-primary' download>📥 Download CSV (" + String(_fileSizeBytes / 1024) + " KB)</a>";
        html += "<a href='/clear' class='btn btn-danger' onclick='return confirm(\"Clear stored Flash samples?\")'>🗑 Clear Flash Data</a>";
        html += "</div>";
        html += "<div class='stats'>";
        html += "<div>💾 <b>Storage:</b> Flash LittleFS Non-Volatile | <b>File Size:</b> " + String(_fileSizeBytes / 1024.0, 1) + " KB | <b>Logged Samples:</b> " + String(_sampleCount) + "</div>";
        html += "<div>⏱ <b>Duration Logged:</b> " + String((_sampleCount) / 3600.0, 2) + " hours | <b>Latest Sample #:</b> " + String(_latestSample.sampleIndex) + " | <b>AP IP:</b> " + _apIP.toString() + "</div>";
        html += "</div>";
        html += "</div></body></html>";

        _server.send(200, "text/html", html);
    }

    void handleCsvDownload()
    {
        if (!LittleFS.exists(CSV_FILE_PATH))
        {
            _server.send(404, "text/plain", "No CSV data file found");
            return;
        }

        File f = LittleFS.open(CSV_FILE_PATH, "r");
        if (!f)
        {
            _server.send(500, "text/plain", "Failed to open CSV file");
            return;
        }

        _server.sendHeader("Content-Disposition", "attachment; filename=\"science_data.csv\"");
        _server.streamFile(f, "text/csv");
        f.close();
    }

    void handleJsonData()
    {
        String json = "{";
        json += "\"timestamp_ms\":" + String(_latestSample.timestampMs) + ",";
        json += "\"sample_index\":" + String(_latestSample.sampleIndex) + ",";
        json += "\"sensor_1\":" + String(_latestSample.sensor1) + ",";
        json += "\"sensor_2\":" + String(_latestSample.sensor2) + ",";
        json += "\"sensor_3\":" + String(_latestSample.sensor3) + ",";
        json += "\"humidity\":" + String(_latestSample.humidity) + ",";
        json += "\"file_size_bytes\":" + String(_fileSizeBytes) + ",";
        json += "\"total_samples\":" + String(_sampleCount);
        json += "}";

        _server.send(200, "application/json", json);
    }

    void handleClear()
    {
        this->createEmptyCsvFile();
        _server.sendHeader("Location", "/");
        _server.send(303);
    }

    const char* _apSsid;
    const char* _apPass;
    WebServer _server;
    IPAddress _apIP;

    size_t _sampleCount = 0;
    size_t _fileSizeBytes = 0;
    SensorSample _latestSample = {0, 0, -1, -1, -1, 0};
};

#endif  // WIFI_SCIENCE_LOGGER_HPP
