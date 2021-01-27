#include <memory>
#include <string>
#include <unordered_set>
#include <vector>
#include <sys/syscall.h>
#include <sys/types.h>
#include <unistd.h>


#include "common/switchboard.hpp"
#include "sqlite_wrapper.hpp"

namespace ILLIXR {

	pid_t gettid(void) {
		return static_cast<pid_t>(syscall(SYS_gettid));
	}

	static const sqlite::schema frame_schema {{
		{"tid", sqlite::type::INTEGER},
		{"id", sqlite::type::INTEGER},
		{"function_name", sqlite::type::INTEGER},
		{"file_name", sqlite::type::INTEGER},
		{"line", sqlite::type::INTEGER},
		{"caller", sqlite::type::INTEGER},
		{"prev", sqlite::type::INTEGER},
		{"youngest_callee", sqlite::type::INTEGER},
		{"wall_start", sqlite::type::INTEGER},
		{"wall_stop", sqlite::type::INTEGER},
		{"cpu_start", sqlite::type::INTEGER},
		{"cpu_stop", sqlite::type::INTEGER},
		{"info0", sqlite::type::INTEGER},
		{"info1", sqlite::type::INTEGER},
		{"info2", sqlite::type::TEXT},
		{"epoch", sqlite::type::INTEGER},
	}};

	class frame_logger {
	private:
		size_t tid;
		sqlite::database database;
		sqlite::table finished_table;
		// sqlite::table unfinished_table;
		sqlite::table string_table;
		size_t epoch;
		std::unordered_set<const char*> strings;

		static boost::filesystem::path get_filename(size_t tid) {
			boost::filesystem::path path = (boost::filesystem::path{"metrics"} / "frames" / std::to_string(tid)).replace_extension(".sqlite");
			boost::filesystem::create_directory(path.parent_path());
			std::cerr << path.string() << std::endl;
			assert(boost::filesystem::is_directory(path.parent_path()));
			return path;
		}

	public:
		frame_logger(pid_t tid_)
			: tid{static_cast<size_t>(tid_)}
			, database{get_filename(tid), true}
			, finished_table{database.create_table("finished", sqlite::schema{frame_schema})}
			// , unfinished_table{database.create_table("unfinished", sqlite::schema{frame_schema})}
			, string_table{database.create_table("strings", {{
				{"address", sqlite::type::INTEGER},
				{"string", sqlite::type::TEXT},
			}})}
		{ }

		void process(cpu_timer::Frames&& finished) {
			std::vector<std::vector<sqlite::value>> string_rows;
			std::vector<std::vector<sqlite::value>> frame_rows;

			for (const cpu_timer::Frame& frame : finished) {
				for (size_t j = 0; j < 2; ++j) {
					const char* string = j == 0 ? frame.get_file_name() : frame.get_function_name();
					if (strings.count(string) == 0) {
						strings.insert(string);
						string_rows.emplace_back(std::vector<sqlite::value>{
							sqlite::value{static_cast<uint64_t>(reinterpret_cast<intptr_t>(string))},
							sqlite::value{std::string{string ? string : ""}}
						});
					}
				}
				frame_rows.emplace_back(std::vector<sqlite::value>{
					sqlite::value{tid},
					sqlite::value{frame.get_index()},
					sqlite::value{static_cast<uint64_t>(reinterpret_cast<intptr_t>(frame.get_function_name()))},
					sqlite::value{static_cast<uint64_t>(reinterpret_cast<intptr_t>(frame.get_file_name()))},
					sqlite::value{frame.get_line()},
					sqlite::value{frame.get_caller_index()},
					sqlite::value{frame.get_prev_index()},
					sqlite::value{frame.get_youngest_callee_index()},
					sqlite::value{cpu_timer::detail::get_ns(frame.get_start_wall())},
					sqlite::value{cpu_timer::detail::get_ns(frame.get_stop_wall())},
					sqlite::value{cpu_timer::detail::get_ns(frame.get_start_cpu())},
					sqlite::value{cpu_timer::detail::get_ns(frame.get_stop_cpu())},
					sqlite::value{size_t(0)},
					sqlite::value{size_t(0)},
					sqlite::value{std::string{""}},
					sqlite::value{epoch},
				});
			}
			finished_table.bulk_insert(std::move(frame_rows));
			string_table.bulk_insert(std::move(string_rows));
		}
	};

	class frame_logger_container : public cpu_timer::CallbackType {
	private:
		std::mutex frame_logger_mutex;
		std::unordered_map<std::thread::id, std::unique_ptr<frame_logger>> frame_loggers;
	protected:
		virtual void thread_start(cpu_timer::Stack& stack) override {
			std::lock_guard<std::mutex> lock {frame_logger_mutex};
			frame_loggers.try_emplace(stack.get_id(), std::make_unique<frame_logger>(stack.get_native_handle()));
		}
		virtual void thread_in_situ(cpu_timer::Stack&) override { }
		virtual void thread_stop(cpu_timer::Stack& stack) override {
			std::lock_guard<std::mutex> lock {frame_logger_mutex};
			frame_loggers.at(stack.get_id())->process(stack.drain_finished());
		}
	};

	static void setup_frame_logger() {
		cpu_timer::get_process().set_callback(std::make_unique<frame_logger_container>());
		cpu_timer::get_process().set_log_period(cpu_timer::CpuNs{0});
		// cpu_timer::get_process().callback_once();
		cpu_timer::get_process().set_enabled(true);
	}
}
