#pragma once

#include <utility>

#include "phonebook.hpp"
#include "record_logger.hpp"


namespace ILLIXR
{

/*
 * This gets included, but it is functionally 'private'. Hence the double-underscores.
 */
const record_header __plugin_start_header {
    "plugin_name",
    {
        { "plugin_id", typeid(std::size_t) },
        { "plugin_name", typeid(std::string) },
    },
};

/**
 * @brief A dynamically-loadable plugin for Spindle.
 */
class plugin
{
public:
    /**
     * @brief A method which Spindle calls when it starts the component.
     */
    virtual void start()
    {
        record_logger_->log(record { __plugin_start_header,
                                     {
                                         { id },
                                         { name },
                                     } });
    }

    /**
     * @brief A method which Spindle calls when it starts the component.
     *
     * This is necessary because the parent class might define some actions that need to be
     * taken prior to destructing the derived class. For example, threadloop must halt and join
     * the thread before the derived class can be safely destructed. However, the derived
     * class's destructor is called before its parent (threadloop), so threadloop doesn't get a
     * chance to join the thread before the derived class is destroyed, and the thread accesses
     * freed memory. Instead, we call plugin->stop manually before destrying anything.
     */
    virtual void stop() { }

    plugin(std::string name_, phonebook* pb_)
        : name { std::move(name_) }
        , pb { pb_ }
        , record_logger_ { pb->lookup_impl<record_logger>() }
        , gen_guid_ { pb->lookup_impl<gen_guid>() }
        , id { gen_guid_->get() }
    { }

    virtual ~plugin() = default;

    auto get_name() -> std::string { return name; }

protected:
    std::string                          name;
    const phonebook*                     pb;
    const std::shared_ptr<record_logger> record_logger_;
    const std::shared_ptr<gen_guid>      gen_guid_;
    const std::size_t                    id;
};


template <typename T> auto this_plugin_factory(phonebook* pb) -> plugin*
{
    T* obj = new T { typeid(T).name(), pb };
    return obj;
}

using plugin_factory_ptr = plugin* (*)(phonebook*);

#ifndef PLUGIN_MAIN
#define PLUGIN_MAIN(plugin_class) ILLIXR::plugin_factory_ptr this_plugin_factory = &ILLIXR::this_plugin_factory<plugin_class>
#endif /// PLUGIN_MAIN

}
