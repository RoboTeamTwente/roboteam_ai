#include <string>
#include <memory>
#include "roboteam_msgs/StringEntry.h"
#include "roboteam_msgs/BoolEntry.h"
#include "roboteam_msgs/Int32Entry.h"
#include "roboteam_msgs/Float64Entry.h"
#include "roboteam_msgs/Blackboard.h"
#include "Blackboard.hpp"

namespace bt {

Blackboard::Blackboard(const roboteam_msgs::Blackboard &msg) {
  fromMsg(msg);
}

void Blackboard::SetBool(std::string key, bool value) { bools[key] = value; }

bool Blackboard::GetBool(std::string key) {
  if (bools.find(key)==bools.end()) {
    bools[key] = false;
  }
  return bools[key];
}

bool Blackboard::HasBool(std::string key) const { return bools.find(key)!=bools.end(); }

void Blackboard::SetInt(std::string key, int value) { ints[key] = value; }

int Blackboard::GetInt(std::string key) {
  if (ints.find(key)==ints.end()) {
    ints[key] = 0;
  }
  return ints[key];
}

bool Blackboard::HasInt(std::string key) const { return ints.find(key)!=ints.end(); }

void Blackboard::SetFloat(std::string key, float value) { floats[key] = value; }

float Blackboard::GetFloat(std::string key) {
  if (floats.find(key)==floats.end()) {
    floats[key] = 0.0f;
  }
  return floats[key];
}

bool Blackboard::HasFloat(std::string key) const { return floats.find(key)!=floats.end(); }

void Blackboard::SetDouble(std::string key, double value) { doubles[key] = value; }

double Blackboard::GetDouble(std::string key) {
  if (doubles.find(key)==doubles.end()) {
    doubles[key] = 0.0f;
  }
  return doubles[key];
}

bool Blackboard::HasDouble(std::string key) const { return doubles.find(key)!=doubles.end(); }

void Blackboard::SetString(std::string key, std::string value) { strings[key] = value; }

std::string Blackboard::GetString(std::string key) {
  if (strings.find(key)==strings.end()) {
    strings[key] = "";
  }
  return strings[key];
}

bool Blackboard::HasString(std::string key) const { return strings.find(key)!=strings.end(); }

using Ptr = std::shared_ptr<Blackboard>;

roboteam_msgs::Blackboard Blackboard::toMsg() {
  roboteam_msgs::Blackboard msg;

  for (const auto &i : bools) {
    roboteam_msgs::BoolEntry entry;
    entry.name = i.first;
    entry.value = i.second;
    msg.bools.push_back(entry);
  }

  for (const auto &i : doubles) {
    roboteam_msgs::Float64Entry entry;
    entry.name = i.first;
    entry.value = i.second;
    msg.doubles.push_back(entry);
  }

  for (const auto &i : strings) {
    roboteam_msgs::StringEntry entry;
    entry.name = i.first;
    entry.value = i.second;
    msg.strings.push_back(entry);
  }

  for (const auto &i : ints) {
    roboteam_msgs::Int32Entry entry;
    entry.name = i.first;
    entry.value = i.second;
    msg.ints.push_back(entry);
  }

  return msg;
}

void Blackboard::fromMsg(const roboteam_msgs::Blackboard &msg) {
  for (const roboteam_msgs::BoolEntry &booleanEntry : msg.bools) {
    SetBool(booleanEntry.name, booleanEntry.value);
  }

  for (const roboteam_msgs::StringEntry &stringEntry : msg.strings) {
    SetString(stringEntry.name, stringEntry.value);
  }

  for (const roboteam_msgs::Int32Entry &intEntry : msg.ints) {
    SetInt(intEntry.name, intEntry.value);
  }

  for (const roboteam_msgs::Float64Entry &doubleEntry : msg.doubles) {
    SetDouble(doubleEntry.name, doubleEntry.value);
  }
}

const std::map<std::string, bool> Blackboard::getBools() {
  return bools;
}

const std::map<std::string, int> Blackboard::getInts() {
  return ints;
}

const std::map<std::string, float> Blackboard::getFloats() {
  return floats;
}

const std::map<std::string, double> Blackboard::getDoubles() {
  return doubles;
}

const std::map<std::string, std::string> Blackboard::getStrings() {
  return strings;
}

std::string Blackboard::toString() {
  std::stringstream ss;
  ss << std::endl;

  if (!bools.empty()) {
    ss << "  BOOLS  ";
    for (auto i : bools) {
      ss << " | " << i.first << " = " << i.second;
    }
    ss << std::endl;
  }

  if (!ints.empty()) {
    ss << "  INTS   ";
    for (auto i : ints) {
      ss << " | " << i.first << " = " << i.second;
    }
    ss << std::endl;
  }

  if (!floats.empty()) {
    ss << "  FLOATS ";
    for (auto i : floats) {
      ss << " | " << i.first << " = " << i.second;
    }
    ss << std::endl;
  }

  if (!doubles.empty()) {
    ss << "  DOUBLES";
    for (auto i : doubles) {
      ss << " | " << i.first << " = " << i.second;
    }
    ss << std::endl;
  }

  if (!strings.empty()) {
    ss << "  STRINGS";
    for (auto i : strings) {
      ss << " | " << i.first << " = " << i.second;
    }
    ss << std::endl;
  }

  return ss.str();
}

std::string Blackboard::toTestX() {
  std::stringstream ss;

  for (auto i : bools) {
    ss << " bool:" << i.first << "=" << i.second;
  }

  for (auto i : ints) {
    ss << " int:" << i.first << "=" << i.second;
  }

  // TODO check if floats or doubles are both needed
  for (auto i : floats) {
    ss << " double:" << i.first << "=" << i.second;
  }

  for (auto i : doubles) {
    ss << " double:" << i.first << "=" << i.second;
  }

  for (auto i : strings) {
    ss << " string:" << i.first << "=" << i.second;
  }
  return ss.str();
}

} // bt
