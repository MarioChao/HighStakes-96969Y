#pragma once

#include <map>
#include <string>
#include <functional>


namespace aespa_lib {
namespace datas {


template <typename T>
class NamedStorage {
public:
	NamedStorage() {}

	std::shared_ptr<T> getStored(std::string key) {
		if (storage_map.find(key) == storage_map.end()) {
			printf("Key %s not found in storage!\n", key.c_str());
			return nullptr;
		}
		return storage_map[key];
	}

	NamedStorage<T> &store(std::string key, T object) {
		if (storage_map.find(key) != storage_map.end()) {
			printf("Key %s already exists in storage!\n", key.c_str());
			return *this;
		}
		storage_map[key] = std::shared_ptr<T>(new T(object));
		return *this;
	}

	void clear() {
		storage_map.clear();
	}

private:
	std::map<std::string, std::shared_ptr<T>> storage_map;
};


}
}
