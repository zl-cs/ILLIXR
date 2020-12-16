#include <thread>
#include <chrono>
#include "common/runtime.hpp"
#include "common/extended_window.hpp"
#include "common/dynamic_lib.hpp"
#include "common/plugin.hpp"
#include "common/global_module_defs.hpp"
#include "switchboard_impl.hpp"
#include "stdout_record_logger.hpp"
#include "noop_record_logger.hpp"
#include "sqlite_record_logger.hpp"
#include "common/global_module_defs.hpp"


using namespace ILLIXR;


class runtime_impl : public runtime {
public:
	runtime_impl(GLXContext appGLCtx)
		: cr{std::make_shared<const_registry>()}
		, _m_glx_fb_width{cr->FB_WIDTH.value()}
		, _m_glx_fb_height{cr->FB_WIDTH.value()}
		, _m_run_duration{cr->RUN_DURATION.value()}
	{
		pb.register_impl<const_registry>(cr);
		pb.register_impl<record_logger>(std::make_shared<sqlite_record_logger>());
		pb.register_impl<gen_guid>(std::make_shared<gen_guid>());
		pb.register_impl<switchboard>(create_switchboard(&pb));
		pb.register_impl<xlib_gl_extended_window>(std::make_shared<xlib_gl_extended_window>(_m_glx_fb_width, _m_glx_fb_height, appGLCtx));
	}

	virtual void load_so(const std::vector<std::string>& so_paths) override {
		std::transform(so_paths.cbegin(), so_paths.cend(), std::back_inserter(libs), [](const auto& so_path) {
			return dynamic_lib::create(so_path);
		});

		std::vector<plugin_factory> plugin_factories;
		std::transform(libs.cbegin(), libs.cend(), std::back_inserter(plugin_factories), [](const auto& lib) {
			return lib.template get<plugin* (*) (phonebook*)>("this_plugin_factory");
		});

		std::transform(plugin_factories.cbegin(), plugin_factories.cend(), std::back_inserter(plugins), [this](const auto& plugin_factory) {
			return std::unique_ptr<plugin>{plugin_factory(&pb)};
		});

		std::for_each(plugins.cbegin(), plugins.cend(), [](const auto& plugin) {
			plugin->start();
		});
	}

	virtual void load_so(const std::string_view so) override {
		auto lib = dynamic_lib::create(so);
		plugin_factory this_plugin_factory = lib.get<plugin* (*) (phonebook*)>("this_plugin_factory");
		load_plugin_factory(this_plugin_factory);
		libs.push_back(std::move(lib));
	}

	virtual void load_plugin_factory(plugin_factory plugin_main) override {
		plugins.emplace_back(plugin_main(&pb));
		plugins.back()->start();
	}

	virtual void wait() override {
		while (!terminate.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds{10});
		}
	}

	virtual void stop() override {
		pb.lookup_impl<switchboard>()->stop();
		for (const std::unique_ptr<plugin>& plugin : plugins) {
			plugin->stop();
		}
		terminate.store(true);
	}

	virtual long get_run_duration() const override {
		return _m_run_duration;
	}

	virtual ~runtime_impl() override {
		if (!terminate.load()) {
			std::cerr << "You didn't call stop() before destructing this plugin." << std::endl;
			abort();
		}
	}

private:
	// I have to keep the dynamic libs in scope until the program is dead
	std::vector<dynamic_lib> libs;
	phonebook pb;
	std::vector<std::unique_ptr<plugin>> plugins;
	std::atomic<bool> terminate {false};

	// Constants set at construction-time (lookup from const_registry)
	const std::shared_ptr<const_registry> cr;

	using CR = ILLIXR::const_registry;
	const CR::DECL_FB_WIDTH::type     _m_glx_fb_width;
	const CR::DECL_FB_HEIGHT::type    _m_glx_fb_height;
	const CR::DECL_RUN_DURATION::type _m_run_duration;
};

extern "C" runtime* runtime_factory(GLXContext appGLCtx) {
	return new runtime_impl{appGLCtx};
}
