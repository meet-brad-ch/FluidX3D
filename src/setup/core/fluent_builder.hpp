#pragma once

/**
 * @file fluent_builder.hpp
 * @brief Base class for fluent builder pattern using CRTP
 *
 * This provides a standardized base for all fluent API classes in the setup system.
 * Uses the Curiously Recurring Template Pattern (CRTP) to enable proper method
 * chaining with derived class return types.
 *
 * @par Example:
 * @code
 * class MyConfig : public FluentBuilder<MyConfig> {
 * public:
 *     MyConfig& set_value(int v) {
 *         value_ = v;
 *         return self();
 *     }
 * private:
 *     int value_ = 0;
 * };
 *
 * // Usage:
 * MyConfig config;
 * config.set_value(42);  // Returns MyConfig&
 * @endcode
 */
template<typename Derived>
class FluentBuilder {
protected:
    /**
     * @brief Get reference to derived class for method chaining
     * @return Reference to the derived class instance
     */
    Derived& self() {
        return static_cast<Derived&>(*this);
    }

    /**
     * @brief Get const reference to derived class
     * @return Const reference to the derived class instance
     */
    const Derived& self() const {
        return static_cast<const Derived&>(*this);
    }

    // Protected destructor to prevent slicing through base pointer
    ~FluentBuilder() = default;
};
