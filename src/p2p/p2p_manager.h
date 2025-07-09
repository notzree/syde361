// Note: Leaving this here for now in case we end up building out some belt->phone thing
// Curerntly unused

#pragma once
#include <functional>
#include <string>

enum class ConnectionStatus {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    ERROR
};

struct ConnectionEvent {
    enum Type { CONNECTED, DISCONNECTED, ERROR, DATA_RECEIVED };
    Type type;
    std::string data;
    unsigned long timestamp;
};

class IConnectivityHandler {
public:
    virtual ~IConnectivityHandler() = default;
    virtual bool initialize() = 0;
    virtual bool startPairing() = 0;
    virtual bool stopPairing() = 0;
    virtual bool isConnected() const = 0;
    virtual bool sendData(const std::string& data) = 0;
    virtual ConnectionStatus getStatus() const = 0;
};

class ConnectivityManager {
public:
    using ConnectionEventCallback = std::function<void(const ConnectionEvent&)>;
    
    ConnectivityManager();
    ~ConnectivityManager() = default;
    
    // Initialization
    bool initialize();
    void update();
    
    // Connection management
    bool startPairing();
    bool stopPairing();
    bool isConnected() const;
    
    // Data transmission
    bool sendSensorData(const std::string& data);
    bool sendCalibrationData(const std::string& data);
    
    // Event handling
    void setEventCallback(ConnectionEventCallback callback);
    
    // Handler management
    void setBluetoothHandler(std::unique_ptr<IConnectivityHandler> handler);
    void setWifiHandler(std::unique_ptr<IConnectivityHandler> handler);
    
private:
    std::unique_ptr<IConnectivityHandler> bluetoothHandler_;
    std::unique_ptr<IConnectivityHandler> wifiHandler_;
    ConnectionEventCallback eventCallback_;
    ConnectionStatus currentStatus_;
};