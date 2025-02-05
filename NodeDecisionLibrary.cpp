#include "NodeDecisionLibrary.h"
#include <ArduinoJson.h>
#include <queue>
#include <set>

// Constructor
NodeDecisionLibrary::NodeDecisionLibrary()
{
    // Boolean Logic Nodes
    nodeLogicMap[1] = [](const std::vector<bool> &inputs)
    { return !inputs[0]; }; // NOT
    nodeLogicMap[2] = [](const std::vector<bool> &inputs)
    { return inputs[0] && inputs[1]; }; // AND
    nodeLogicMap[3] = [](const std::vector<bool> &inputs)
    { return inputs[0] || inputs[1]; }; // OR
    nodeLogicMap[4] = [](const std::vector<bool> &inputs)
    { return inputs[0] ^ inputs[1]; }; // XOR
    nodeLogicMap[5] = [](const std::vector<bool> &inputs)
    { return !(inputs[0] || inputs[1]); }; // NOR
    nodeLogicMap[6] = [](const std::vector<bool> &inputs)
    { return !(inputs[0] && inputs[1]); }; // NAND
    nodeLogicMap[7] = [](const std::vector<bool> &inputs)
    { return !(inputs[0] ^ inputs[1]); }; // XNOR

    // Mathematical Nodes (Function)
    mathNodeMap[8] = [](const std::vector<double> &inputs)
    { return inputs[0] + inputs[1]; }; // ADD
    mathNodeMap[9] = [](const std::vector<double> &inputs)
    { return inputs[0] - inputs[1]; }; // SUBTRACT
    mathNodeMap[10] = [](const std::vector<double> &inputs)
    { return inputs[0] * inputs[1]; }; // MULTIPLY
    mathNodeMap[11] = [](const std::vector<double> &inputs)
    { return inputs[1] != 0 ? inputs[0] / inputs[1] : 0; }; // DIVIDE (Avoid Zero Division)
    mathNodeMap[12] = [](const std::vector<double> &inputs)
    { return pow(inputs[0], inputs[1]); }; // POWER
    mathNodeMap[13] = [](const std::vector<double> &inputs)
    { return log(inputs[0]); }; // LOGARITHM
    mathNodeMap[14] = [](const std::vector<double> &inputs)
    { return sqrt(inputs[0]); }; // SQUARE ROOT
    mathNodeMap[15] = [](const std::vector<double> &inputs)
    { return fabs(inputs[0]); }; // ABSOLUTE
    mathNodeMap[16] = [](const std::vector<double> &inputs)
    { return exp(inputs[0]); }; // EXPONENT

    // Comparison Nodes (Return Boolean)
    mathNodeMap[17] = [](const std::vector<double> &inputs)
    { return std::min(inputs[0], inputs[1]); }; // MIN
    mathNodeMap[18] = [](const std::vector<double> &inputs)
    { return std::max(inputs[0], inputs[1]); }; // MAX
    mathNodeMap[19] = [](const std::vector<double> &inputs)
    {
        return (inputs[0] < inputs[1]) ? 1.0 : 0.0;
    }; // LESS THAN

    mathNodeMap[20] = [](const std::vector<double> &inputs)
    {
        return (inputs[0] > inputs[1]) ? 1.0 : 0.0;
    }; // GREATER THAN
    mathNodeMap[21] = [](const std::vector<double> &inputs)
    { return (inputs[0] <= inputs[1]) ? 1.0 : 0.0; }; // LESS THAN OR EQUAL
    mathNodeMap[22] = [](const std::vector<double> &inputs)
    { return (inputs[0] >= inputs[1]) ? 1.0 : 0.0; }; // GREATER THAN OR EQUAL
    mathNodeMap[23] = [](const std::vector<double> &inputs)
    { return (inputs[0] == inputs[1]) ? 1.0 : 0.0; }; // EQUAL
    mathNodeMap[24] = [](const std::vector<double> &inputs)
    { return (inputs[0] != inputs[1]) ? 1.0 : 0.0; }; // NOT EQUAL

    // Rounding Nodes (Function)
    mathNodeMap[25] = [](const std::vector<double> &inputs)
    { return round(inputs[0]); }; // ROUND
    mathNodeMap[26] = [](const std::vector<double> &inputs)
    { return floor(inputs[0]); }; // FLOOR
    mathNodeMap[27] = [](const std::vector<double> &inputs)
    { return ceil(inputs[0]); }; // CEIL

    nodeLogicMap[28] = [](const std::vector<bool> &inputs)
    { return inputs[0]; }; // Final Node
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
            if (input.containsKey("d") && !input["d"].isNull())
            {
                inputData.data = input["d"].as<std::string>();
            }
            else
            {
                inputData.data = "null";
            }
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

    for (const auto &node : nodesForDevice)
    {
        for (const auto &input : node.inputs)
        {
            validInputIds.insert(input.id);
        }
        for (const auto &output : node.outputs)
        {
            validOutputIds.insert(output.id);
        }
    }

    std::vector<RelationshipData> relationshipsForDevice;
    for (JsonObject relationship : relationshipsArray)
    {
        RelationshipData relationshipData;
        relationshipData.id = relationship["id"];
        relationshipData.inputId = relationship["i"];
        relationshipData.outputId = relationship["o"];
        relationshipData.configId = relationship["c"];

        if (validInputIds.count(relationshipData.inputId) > 0 &&
            validOutputIds.count(relationshipData.outputId) > 0)
        {
            relationshipsForDevice.push_back(relationshipData);
        }
        else
        {
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
    std::map<int, std::string> calculatedDeviceValues;

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
            std::string inputValue = input.data; // Default value

            bool hasRelationship = false;
            for (const auto &relationship : relationships)
            {
                if (relationship.inputId == input.id)
                {
                    hasRelationship = true;
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
            // If no relationship exists, keep the default value
            if (!hasRelationship)
            {
            }

            inputValues.push_back(inputValue);
        }

        // Handle direct device values
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
        // Handle Final Node
        else if (node->availableId == 28)
        {

            if (!node->inputs.empty())
            {
                bool booleanValue = convertToBool(node->inputs[0].data);
                // calculatedOutputValues[node->id] = booleanValue ? "true" : "false";
                calculatedDeviceValues[node->id] = booleanValue ? "true" : "false";
            }
        }
        // Boolean Logic Nodes
        else if (nodeLogicMap.find(node->availableId) != nodeLogicMap.end())
        {
            std::vector<bool> boolInputs;
            for (const auto &val : inputValues)
            {
                std::string lowerVal = val;
                std::transform(lowerVal.begin(), lowerVal.end(), lowerVal.begin(), ::tolower);

                // Convert string to boolean (including numeric values)
                bool booleanValue = false;
                if (lowerVal == "true" || lowerVal == "1" || lowerVal == "yes" || lowerVal == "on")
                {
                    booleanValue = true;
                }
                else if (lowerVal == "false" || lowerVal == "0" || lowerVal == "no" || lowerVal == "off")
                {
                    booleanValue = false;
                }
                else
                {
                    try
                    {
                        double numericValue = std::stod(val);
                        booleanValue = (numericValue != 0.0); // Nonzero numbers are true
                    }
                    catch (...)
                    {
                        booleanValue = false; // If conversion fails, assume false
                    }
                }
                boolInputs.push_back(booleanValue);
            }

            bool result = nodeLogicMap[node->availableId](boolInputs);
            std::string resultStr = result ? "true" : "false";

            for (auto &output : node->outputs)
            {
                output.data = resultStr;
                calculatedOutputValues[output.id] = resultStr;
            }
        }
        // Math / Comparison Nodes
        else if (mathNodeMap.find(node->availableId) != mathNodeMap.end())
        {
            std::vector<double> numericInputs;
            for (const auto &val : inputValues)
            {
                try
                {
                    double num = std::stod(val);
                    numericInputs.push_back(num);
                }
                catch (...)
                {
                    numericInputs.push_back(0.0);
                }
            }

            double result = mathNodeMap[node->availableId](numericInputs);
            std::string resultStr = std::to_string(result);
            if (result == 1.0)
                resultStr = "1.0";
            for (auto &output : node->outputs)
            {
                output.data = resultStr;
                calculatedOutputValues[output.id] = resultStr;
            }
        }
        else
        {
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

    // Return boolean result if target node outputs boolean
    if (targetNode && targetNode->availableId == 28 && !targetNode->inputs.empty())
    {
        return convertToBool(targetNode->inputs[0].data);
    }

    if (!targetNode->outputs.empty())
    {
        std::string resultStr = calculatedOutputValues[targetNode->outputs[0].id];

        // If output is boolean, return boolean result
        if (resultStr == "true" || resultStr == "false")
        {
            return resultStr == "true";
        }
    }

    return false;
}

bool NodeDecisionLibrary::convertToBool(const std::string &value)
{
    // Trim leading and trailing spaces
    std::string trimmedValue = value;
    trimmedValue.erase(0, trimmedValue.find_first_not_of(" \t\n\r"));
    trimmedValue.erase(trimmedValue.find_last_not_of(" \t\n\r") + 1);

    // Convert string to lowercase
    std::string lowerValue = trimmedValue;
    std::transform(lowerValue.begin(), lowerValue.end(), lowerValue.begin(), ::tolower);

    // Handle string values explicitly
    if (lowerValue == "true" || lowerValue == "1" || lowerValue == "yes" || lowerValue == "on")
    {
        return true;
    }
    if (lowerValue == "false" || lowerValue == "0" || lowerValue == "no" || lowerValue == "off")
    {
        return false;
    }

    // Try converting numeric values
    try
    {
        double numericValue = std::stod(trimmedValue);
        return numericValue != 0.0; // Any non-zero value is true
    }
    catch (...)
    {
        return false; // If conversion fails, assume false
    }
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
   // Serial.print(valueString.c_str());

    DynamicJsonDocument doc(16484);
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
        std::string valueType = "";
        if (sensor["value"].is<bool>())
        {
            valueStr = sensor["value"].as<bool>() ? "true" : "false";
            valueType = "bool";
        }
        else if (sensor["value"].is<float>() || sensor["value"].is<int>() || sensor["value"].is<double>() || sensor["value"].is<JsonVariant>())
        {
            double numValue = sensor["value"].as<double>(); // Read as double
            valueStr = std::to_string(numValue);

            // Type detection based on numerical value
            if (numValue == static_cast<int>(numValue))
                valueType = "int";
            else
                valueType = "double";
        }
        else if (sensor["value"].is<const char *>())
        {
            valueStr = sensor["value"].as<const char *>();
            valueType = "string";
        }
        else
        {
            valueStr = "unknown";
            valueType = "unknown";
        }

        deviceValues[deviceId] = valueStr;
        debugPrint("\n Updated deviceValues: Device ID %d -> Value %s | Type: %s \n",
                   deviceId, valueStr.c_str(), valueType.c_str());
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
    }
    else
    {
        Serial.printf("Device ID %d: Waiting for debounce duration. Current state: %s\n", deviceId, newValue ? "true" : "false");
    }
}
void NodeDecisionLibrary::setDebounceDuration(unsigned long duration)
{
    debounceDuration = duration;
    debugPrint("Debounce duration set to %lu milliseconds.\n", debounceDuration);
}

void NodeDecisionLibrary::setCallback(std::function<void(int, bool)> callbackFunc)
{
    callback = callbackFunc;
}
int NodeDecisionLibrary::getVersion()
{
    return version;
}
