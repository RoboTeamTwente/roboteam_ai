#pragma once

#include <string>
#include <memory>
#include <map>
#include "roboteam_utils/Vector2.h"
#include <gtest/gtest.h>
#include <gtest/gtest_prod.h>

namespace bt {

class Blackboard {
        FRIEND_TEST(BTBlackBoardTest, Blackboard);

    public:
        // Default constructors enabled
        Blackboard() = default;
        Blackboard(const Blackboard &) = default;

        Blackboard(Blackboard &&) = default;

        using Ptr = std::shared_ptr<Blackboard>;

        Blackboard &operator=(const Blackboard &) = default;

        Blackboard &operator=(Blackboard &&) = default;

        virtual ~Blackboard() = default;

        void setBool(std::string key, bool value);
        bool getBool(std::string key);
        bool hasBool(std::string key) const;

        void setInt(std::string key, int value);
        int getInt(std::string key);
        bool hasInt(std::string key) const;

        void setFloat(std::string key, float value);
        float getFloat(std::string key);
        bool hasFloat(std::string key) const;

        void setDouble(std::string key, double value);
        double getDouble(std::string key);
        bool hasDouble(std::string key) const;

        void setString(std::string key, std::string value);
        std::string getString(std::string key);
        bool hasString(std::string key) const;

        rtt::Vector2 getVector2(std::string key);
        void setVector2(std::string key, rtt::Vector2 value);
        bool hasVector2(std::string key) const;

    protected:

        std::map<std::string, int> ints;
        std::map<std::string, float> floats;
        std::map<std::string, bool> bools;
        std::map<std::string, std::string> strings;
        std::map<std::string, rtt::Vector2> vectors;
        std::map<std::string, double> doubles;

};

} // bt
