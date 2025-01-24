#ifndef NODE_DECISION_LIBRARY_H
#define NODE_DECISION_LIBRARY_H

#include <ArduinoJson.h>
#include <map>
#include <vector>
#include <string>
#include <functional>
#include <queue>
#include <Arduino.h>
#include <functional>
#include <chrono> 

class NodeDecisionLibrary
{
public:
    NodeDecisionLibrary();
    bool decodeLogicData(const String &jsonPayload, int deviceId);
    void updateDeviceValues(String &valueString);
    void printDecodedData(int deviceId) const;
    void isDebug(bool enabled); 
    void setCallback(std::function<void(int, bool)> callback);  
    void processPendingChanges();
    void setDebounceDuration(unsigned long duration);

private:
    struct InputData
    {
        int id;
        std::string dataType;
        std::string data;
    };

    struct OutputData
    {
        int id;
        std::string dataType;
        std::string data;
        int deviceId;
        int configId;
    };

    struct NodeData
    {
        int id;
        int availableId;
        std::string kind;
        std::string data;
        std::vector<InputData> inputs;
        std::vector<OutputData> outputs;
    };

    struct RelationshipData
    {
        int id;
        int inputId;
        int outputId;
        int configId;
    };

    std::map<int, std::vector<NodeData>> deviceNodes;
    std::map<int, std::vector<RelationshipData>> deviceRelationships;
    std::map<int, std::map<int, std::string>> deviceDIds;
    std::map<int, std::string> deviceValues;
    std::function<void(int, bool)> callback;
    std::map<int, std::function<bool(const std::vector<bool> &)>> nodeLogicMap;
    std::map<int, unsigned long> lastTriggerTime; 
    std::map<int, bool> pendingValues;           

    bool debugEnabled = false;
    unsigned long debounceDuration = 10000;       // 10 seconds debounce duration (in milliseconds)

    std::vector<int> topologicalSort(int deviceId);
    bool evaluateNodeInput(int deviceId, int targetNodeId);
    void debugPrint(const char *format, ...);
    void processDeviceChange(int deviceId, bool newValue);  
    
};

#endif
