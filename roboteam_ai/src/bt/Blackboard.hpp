#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include "roboteam_msgs/Blackboard.h"

namespace bt {

    class Blackboard {
    public:
        // Default constructors enabled
        Blackboard() = default;

        Blackboard(const Blackboard &) = default;

        Blackboard(Blackboard &&) = default;

        Blackboard &operator=(const Blackboard &) = default;

        Blackboard &operator=(Blackboard &&) = default;

        virtual ~Blackboard() = default;

        Blackboard(const roboteam_msgs::Blackboard &msg);

        void SetBool(std::string key, bool value);

        bool GetBool(std::string key);

        bool HasBool(std::string key) const;

        void SetInt(std::string key, int value);

        int GetInt(std::string key);

        bool HasInt(std::string key) const;

        void SetFloat(std::string key, float value);

        float GetFloat(std::string key);

        bool HasFloat(std::string key) const;

        void SetDouble(std::string key, double value);

        double GetDouble(std::string key);

        bool HasDouble(std::string key) const;

        void SetString(std::string key, std::string value);

        std::string GetString(std::string key);

        bool HasString(std::string key) const;

        using Ptr = std::shared_ptr<Blackboard>;

        roboteam_msgs::Blackboard toMsg();

        void fromMsg(const roboteam_msgs::Blackboard &msg);

        const std::map<std::string, bool> getBools();

        const std::map<std::string, int> getInts();

        const std::map<std::string, float> getFloats();

        const std::map<std::string, double> getDoubles();

        const std::map<std::string, std::string> getStrings();

        std::string toString();

        std::string toTestX();

    protected:
        std::map<std::string, bool> bools;
        std::map<std::string, int> ints;
        std::map<std::string, float> floats;
        std::map<std::string, double> doubles;
        std::map<std::string, std::string> strings;

    };

} // bt
