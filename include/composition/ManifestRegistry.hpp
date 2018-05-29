#ifndef COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP

#include <string>
#include <unordered_map>
#include <composition/Manifest.hpp>

namespace composition {

	typedef std::function<void(Manifest)> PatchFunction; //TODO ApplyManifestCallback

	class ManifestRegistry {
	public:
		static void Add(Manifest m, PatchFunction p) {
			RegisteredManifests()->push_back(m);
			(*ManifestPatchers())[m] = p;
		}

		static std::vector<Manifest> *GetAll() {
			return RegisteredManifests();
		};

		static void Remove(uintptr_t idx) {
			for (auto it = RegisteredManifests()->begin(), it_end = RegisteredManifests()->end(); it != it_end; it++) {
				if (it->idx == idx) {
					ManifestPatchers()->erase(*it);
					it = RegisteredManifests()->erase(it);
				}
			}
		}

		static PatchFunction GetPatcher(Manifest m) {
			return (*ManifestPatchers())[m];
		}

		static std::unordered_map<Manifest, PatchFunction> *GetAllManifestPatchers() {
			return ManifestPatchers();
		};

	protected:
		static std::vector<Manifest> *RegisteredManifests() {
			static std::vector<Manifest> value = {};
			return &value;
		};

		static std::unordered_map<Manifest, PatchFunction> *ManifestPatchers() {
			static std::unordered_map<Manifest, PatchFunction> value = {};
			return &value;
		};
	};
}

#endif //COMPOSITION_FRAMEWORK_MANIFESTREGISTRY_HPP
