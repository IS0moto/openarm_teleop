// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>
#include <vector>

class YamlLoader {
public:
    explicit YamlLoader(const std::string& filepath) {
        try {
            root_ = YAML::LoadFile(filepath);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load YAML file: " + filepath +
                                     ", error: " + e.what());
        }
    }

    // Get a scalar double value
    double get_double(const std::string& node_name, const std::string& key) const {
        return get_node(node_name, key).as<double>();
    }

    // Get a vector of doubles
    std::vector<double> get_vector(const std::string& node_name, const std::string& key) const {
        return get_node(node_name, key).as<std::vector<double>>();
    }

    // Get a scalar string value
    std::string get_string(const std::string& node_name, const std::string& key) const {
        return get_node(node_name, key).as<std::string>();
    }

    // Get a scalar bool value
    bool get_bool(const std::string& node_name, const std::string& key) const {
        return get_node(node_name, key).as<bool>();
    }

    // Get a scalar int value
    int get_int(const std::string& node_name, const std::string& key) const {
        return get_node(node_name, key).as<int>();
    }

    // Scalar getters with a default when the key (or node) is absent
    double get_double_or(const std::string& node_name, const std::string& key, double def) const {
        return has(node_name, key) ? get_double(node_name, key) : def;
    }
    std::string get_string_or(const std::string& node_name, const std::string& key,
                              const std::string& def) const {
        return has(node_name, key) ? get_string(node_name, key) : def;
    }
    bool get_bool_or(const std::string& node_name, const std::string& key, bool def) const {
        return has(node_name, key) ? get_bool(node_name, key) : def;
    }
    int get_int_or(const std::string& node_name, const std::string& key, int def) const {
        return has(node_name, key) ? get_int(node_name, key) : def;
    }

    // Check if key exists
    bool has(const std::string& node_name, const std::string& key) const {
        return root_[node_name] && root_[node_name][key];
    }

    bool has_node(const std::string& node_name) const { return static_cast<bool>(root_[node_name]); }

private:
    YAML::Node get_node(const std::string& node_name, const std::string& key) const {
        if (!root_[node_name]) {
            throw std::runtime_error("Node '" + node_name + "' not found.");
        }
        if (!root_[node_name][key]) {
            throw std::runtime_error("Key '" + key + "' not found under node '" + node_name + "'.");
        }
        return root_[node_name][key];
    }

    YAML::Node root_;
};
