#include "NodeDecisionLibrary.h"
#include <ArduinoJson.h>
#include <queue>
#include <set>


// Constructor
NodeDecisionLibrary::NodeDecisionLibrary()
{

    nodeLogicMap[1] = [](const std::vector<bool> &inputs)    { return !inputs[0]; }; // NOT
    nodeLogicMap[2] = [](const std::vector<bool> &inputs)    { return inputs[0] && inputs[1]; }; // AND
    nodeLogicMap[3] = [](const std::vector<bool> &inputs)    { return inputs[0] || inputs[1]; }; // OR
    nodeLogicMap[4] = [](const std::vector<bool> &inputs)    { return inputs[0] ^ inputs[1]; }; // XOR
    nodeLogicMap[28] = [](const std::vector<bool> &inputs)    { return inputs[0]; }; // Final Node
}

void NodeDecisionLibrary::isDebug(bool enabled)
{
    debugEnabled = enabled;
}

void NodeDecisionLibrary::debugPrint(const char *format, ...)
{
    if (debugEnabled)
    {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
}
bool NodeDecisionLibrary::decodeLogicData(const String &jsonPayload, int deviceId)
{
    debugPrint("Decoding JSON...\n");
    DynamicJsonDocument doc(16384);
    DeserializationError error = deserializeJson(doc, jsonPayload);

    if (error)
    {
        debugPrint("Failed to parse JSON: %s\n", error.c_str());
        return false;
    }

    JsonObject data = doc["data"];
    JsonArray nodesArray = data["n"];
    JsonArray relationshipsArray = data["r"];

    std::vector<NodeData> nodesForDevice;

    for (JsonObject node : nodesArray)
    {
        NodeData nodeData;
        nodeData.id = node["id"];
        nodeData.availableId = node["aId"];
        nodeData.kind = node["k"].as<std::string>();

        for (JsonObject input : node["i"].as<JsonArray>())
        {
            InputData inputData;
            inputData.id = input["id"];
            inputData.dataType = input["dt"].as<std::string>();
            inputData.data = "null"; // Initialize with null
            nodeData.inputs.push_back(inputData);
        }

        for (JsonObject output : node["o"].as<JsonArray>())
        {
            OutputData outputData;
            outputData.id = output["id"];
            outputData.dataType = output["dt"].as<std::string>();
            outputData.deviceId = output["dId"];
            outputData.configId = output["cId"];
            nodeData.outputs.push_back(outputData);
        }

        nodesForDevice.push_back(nodeData);
    }

    deviceNodes[deviceId] = nodesForDevice;
     // Collect all valid input and output IDs
    std::set<int> validInputIds;
    std::set<int> validOutputIds;

    for (const auto &node : nodesForDevice) {
        for (const auto &input : node.inputs) {
            validInputIds.insert(input.id);
        }
        for (const auto &output : node.outputs) {
            validOutputIds.insert(output.id);
        }
    }

    std::vector<RelationshipData> relationshipsForDevice;
    for (JsonObject relationship : relationshipsArray) {
        RelationshipData relationshipData;
        relationshipData.id = relationship["id"];
        relationshipData.inputId = relationship["i"];
        relationshipData.outputId = relationship["o"];
        relationshipData.configId = relationship["c"];

        if (validInputIds.count(relationshipData.inputId) > 0 &&
            validOutputIds.count(relationshipData.outputId) > 0) {
            relationshipsForDevice.push_back(relationshipData);
        } else {
            debugPrint("Invalid relationship found and removed: ID %d\n", relationshipData.id);
        }
    }
    deviceRelationships[deviceId] = relationshipsForDevice;

    debugPrint("JSON decoding and parsing completed successfully.\n");
    return true;
}

std::vector<int> NodeDecisionLibrary::topologicalSort(int deviceId)
{
    std::map<int, std::vector<int>> graph; 
    std::map<int, int> inDegree;           
    std::vector<int> sortedOrder;     

    auto &relationships = deviceRelationships[deviceId];
    auto &nodes = deviceNodes[deviceId];
  
    std::map<int, int> connectionToNodeMap;
    for (const auto &node : nodes)
    {
        for (const auto &input : node.inputs)
        {
            connectionToNodeMap[input.id] = node.id;
        }
        for (const auto &output : node.outputs)
        {
            connectionToNodeMap[output.id] = node.id;
        }
    }

    for (const auto &relationship : relationships)
    {
        int inputNode = connectionToNodeMap[relationship.inputId];   
        int outputNode = connectionToNodeMap[relationship.outputId]; 

        graph[outputNode].push_back(inputNode); 
        inDegree[inputNode]++;                 
        inDegree[outputNode];                  
    }

    std::queue<int> zeroInDegree;
    debugPrint("Finding nodes with zero in-degree...");
    for (const auto &entry : inDegree)
    {
        if (entry.second == 0)
        {
            zeroInDegree.push(entry.first);
            debugPrint("Node ID %d added to zero in-degree queue\n", entry.first);
        }
    }

    debugPrint("Processing nodes with zero in-degree...");
    while (!zeroInDegree.empty())
    {
        int current = zeroInDegree.front();
        zeroInDegree.pop();
        sortedOrder.push_back(current);
        debugPrint("Processing Node ID %d\n", current);

        for (int dependent : graph[current])
        {
            inDegree[dependent]--;
            debugPrint("Decreased in-degree of Node ID %d to %d\n", dependent, inDegree[dependent]);
            if (inDegree[dependent] == 0)
            {
                zeroInDegree.push(dependent);
                debugPrint("Node ID %d added to zero in-degree queue\n", dependent);
            }
        }
    }

    if (sortedOrder.size() != inDegree.size())
    {
        debugPrint("Error: Graph has cycles, cannot perform topological sort.\n");
        return {};
    }
 
    debugPrint("Topological sort completed. Sorted order:");
    for (int nodeId : sortedOrder)
    {
        debugPrint("%d ", nodeId);
    }
    debugPrint("\n");

    return sortedOrder;
}

bool NodeDecisionLibrary::evaluateNodeInput(int deviceId, int targetNodeId)
{
    auto &nodes = deviceNodes[deviceId];
    auto &relationships = deviceRelationships[deviceId];
    std::map<int, std::string> calculatedOutputValues;
    
    std::function<void(int)> calculateNodeValues = [&](int nodeId)
    {
        if (calculatedOutputValues.find(nodeId) != calculatedOutputValues.end())
        {
            return;
        }
        NodeData *node = nullptr;
        for (auto &n : nodes)
        {
            if (n.id == nodeId)
            {
                node = &n;
                break;
            }
        }
        if (!node)
            return;

        std::vector<std::string> inputValues;
        for (auto &input : node->inputs)
        {
            std::string inputValue = input.data; 
            for (const auto &relationship : relationships)
            {
                if (relationship.inputId == input.id)
                {
                    for (const auto &otherNode : nodes)
                    {
                        for (const auto &output : otherNode.outputs)
                        {
                            if (output.id == relationship.outputId)
                            {
                                calculateNodeValues(otherNode.id); 
                                inputValue = calculatedOutputValues[output.id];
                                input.data = inputValue; 
                            }
                        }
                    }
                }
            }
            inputValues.push_back(inputValue);
        }

        if (node->availableId == 30)
        { 
            for (auto &output : node->outputs)
            {
                if (deviceValues.find(output.deviceId) != deviceValues.end())
                {
                    output.data = deviceValues[output.deviceId];
                    calculatedOutputValues[output.id] = output.data;
                }
            }
        }
        else if (node->availableId == 28)
        { 
            if (!node->inputs.empty())
            {
                calculatedOutputValues[node->id] = node->inputs[0].data;
            }
        }
        else if (nodeLogicMap.find(node->availableId) != nodeLogicMap.end())
        { 
            std::vector<bool> boolInputs;
            for (const auto &val : inputValues)
            {
                boolInputs.push_back(val == "true");
            }
            bool result = nodeLogicMap[node->availableId](boolInputs);
            std::string resultStr = result ? "true" : "false";

            for (auto &output : node->outputs)
            {
                output.data = resultStr;
                calculatedOutputValues[output.id] = resultStr;
            }
        }
    };

    calculateNodeValues(targetNodeId);

    NodeData *targetNode = nullptr;
    for (auto &n : nodes)
    {
        if (n.id == targetNodeId)
        {
            targetNode = &n;
            break;
        }
    }
    if (targetNode)
    {
        debugPrint("Node ID: %d, Inputs: ", targetNodeId);
        for (const auto &input : targetNode->inputs)
        {
            debugPrint("%s ", input.data.c_str());
        }
        debugPrint(", Outputs: ");
        for (const auto &output : targetNode->outputs)
        {
            debugPrint("%s ", output.data.c_str());
        }
         debugPrint("\n");
    }

    if (targetNode && targetNode->availableId == 28 && !targetNode->inputs.empty())
    {
        return targetNode->inputs[0].data == "true";
    }

    return !targetNode->outputs.empty() &&
           calculatedOutputValues[targetNode->outputs[0].id] == "true";
}

void NodeDecisionLibrary::processPendingChanges()
{
    unsigned long currentTime = millis();
    for (auto it = pendingValues.begin(); it != pendingValues.end(); ++it)
    {
        int deviceId = it->first;
        bool newValue = it->second;

        if (currentTime - lastTriggerTime[deviceId] >= debounceDuration)
        {
            if (callback)
            {
                callback(deviceId, newValue);
                debugPrint("Device ID: %d, Applied Pending Value: %s\n", deviceId, newValue ? "true" : "false");
            }
            pendingValues.erase(deviceId);   
            lastTriggerTime.erase(deviceId);
        }
    }
}

void NodeDecisionLibrary::updateDeviceValues(String &valueString)
{
    debugPrint("Updating Device Values...\n");
    debugPrint("Received JSON:\n");
    debugPrint(valueString.c_str()); 

    DynamicJsonDocument doc(16384);
    DeserializationError error = deserializeJson(doc, valueString);

    if (error)
    {
        debugPrint("Failed to parse JSON: ");
        debugPrint(error.c_str());
        return;
    }

    JsonArray sensorArray = doc["sensorArray"].as<JsonArray>();
    for (JsonObject sensor : sensorArray)
    {
        int deviceId = sensor["deviceId"];
        std::string valueStr;

        if (sensor["value"].is<bool>())
        {
            valueStr = sensor["value"].as<bool>() ? "true" : "false";
        }
        else if (sensor["value"].is<int>())
        {
            valueStr = std::to_string(sensor["value"].as<int>());
        }
        else if (sensor["value"].is<float>())
        {
            valueStr = std::to_string(sensor["value"].as<float>());
        }
        else
        {
            valueStr = sensor["value"].as<std::string>();
        }

        deviceValues[deviceId] = valueStr;
        debugPrint("Updated deviceValues: Device ID %d -> Value %s\n", deviceId, valueStr.c_str());
    }

    for (auto it = deviceNodes.begin(); it != deviceNodes.end(); ++it)
    {
        int deviceId = it->first;
        auto &nodes = it->second;

        debugPrint("Processing Device ID: %d\n", deviceId);
        auto sortedNodes = topologicalSort(deviceId);
        debugPrint("Topological sort order:");
        for (int nodeId : sortedNodes)
        {
            debugPrint("%d ", nodeId);
        }
        debugPrint("\n");

        for (int nodeId : sortedNodes)
        {
            debugPrint("Calling evaluateNodeInput for Node ID: %d\n", nodeId);
            bool outputData = evaluateNodeInput(deviceId, nodeId);
            debugPrint("Device ID: %d, Outputs: %s\n", deviceId, outputData ? "true" : "false");

            for (auto &node : nodes)
            {
                if (node.id == nodeId && node.availableId == 28)
                {
                    processDeviceChange(deviceId, outputData);
                }
            }
        }
    }
    debugPrint("Device values updated successfully.\n");
}

void NodeDecisionLibrary::processDeviceChange(int deviceId, bool newValue)
{
    unsigned long currentTime = millis();
    if (pendingValues.count(deviceId) > 0 && pendingValues[deviceId] != newValue)
    {
        Serial.printf("Device ID %d: Oscillating state detected. Ignoring intermediate state.\n", deviceId);
        lastTriggerTime[deviceId] = currentTime; 
        pendingValues[deviceId] = newValue;     
        return;
            }

    if (lastTriggerTime.count(deviceId) == 0 || (currentTime - lastTriggerTime[deviceId] >= debounceDuration))
    {
        lastTriggerTime[deviceId] = currentTime; 
        pendingValues[deviceId] = newValue;  
        if (callback)
        {
            callback(deviceId, newValue); 
        }
        Serial.printf("Device ID %d: Callback triggered with value: %s\n", deviceId, newValue ? "true" : "false");
    }
    else
    {
        Serial.printf("Device ID %d: Waiting for debounce duration. Current state: %s\n",
                      deviceId, newValue ? "true" : "false");
    }
}
void NodeDecisionLibrary::setDebounceDuration(unsigned long duration) {
    debounceDuration = duration;
    debugPrint("Debounce duration set to %lu milliseconds.\n", debounceDuration);
}


void NodeDecisionLibrary::setCallback(std::function<void(int, bool)> callbackFunc)
{
    callback = callbackFunc;
}
