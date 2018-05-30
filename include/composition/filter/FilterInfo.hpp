#ifndef COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
#define COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP

#include <unordered_map>
#include <unordered_set>

namespace composition {

	template<class T>
	class FilterInfo {
	public:
		void add(std::string name, T v) noexcept;

		bool has(std::string name, T v) const noexcept;

		const std::unordered_set<T> &all() const noexcept;

		const std::unordered_set<T> &allByName(std::string name) const noexcept;

		size_t size() const noexcept;

		size_t sizeByName(std::string name) const noexcept;

	private:
		std::unordered_multimap<std::string, T> Values{};
	};
}
#endif //COMPOSITION_FRAMEWORK_FUNCTION_INFO_HPP
