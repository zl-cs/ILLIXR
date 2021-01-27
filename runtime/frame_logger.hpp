#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "common/switchboard.hpp"
#include "sqlite_wrapper.hpp"

namespace ILLIXR {

	class frame_data : public switchboard::event {
	public:
		size_t tid;
		cpu_timer::Frames finished;
		cpu_timer::Frames in_progress;
		frame_data(size_t tid_, cpu_timer::Frames finished_, cpu_timer::Frames in_progress_)
			: tid{tid_}
			, finished{std::move(finished_)}
			, in_progress{std::move(in_progress_)}
		{ }
	};

	static sqlite::schema frame_schema {{
		{"thread", sqlite::type::INTEGER},
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
	}};

	class frame_logger {
	private:
		std::shared_ptr<switchboard> sb;
		std::optional<switchboard::writer<frame_data>> frame_data_writer;
		std::string topic_name;
		sqlite::database database;
		sqlite::table frame_table;
		sqlite::table string_table;
	public:
		frame_logger()
			: topic_name{"frames"}
			, database{std::move((boost::filesystem::path{"common"} / topic_name).replace_extension(".sqlite")), true}
			, frame_table{database.create_table("frames", std::move(frame_schema))}
			, string_table{database.create_table("strings", {{
				{"address", sqlite::type::INTEGER},
				{"string", sqlite::type::TEXT},
			}})}
		{
			cpu_timer::get_process().set_callback([&](const cpu_timer::Stack& stack, cpu_timer::Frames&& finished, const cpu_timer::Frames& in_progress) {
				if (frame_data_writer) {
					auto s = frame_data_writer->allocate(
						reinterpret_cast<size_t>(stack.get_native_handle()),
						finished,
						in_progress
					);
					assert(s.unique());
					frame_data_writer->put(std::move(s));
				} else {
					std::cerr
						<< "cpu_timer callback called before switchboard was up (nowhere to put the data).\n"
						<< "If this happens a lot, we should buffer it until the first callback after switchboard is up.\n";
					abort();
				}
			});
			cpu_timer::get_process().set_log_period(cpu_timer::CpuNs{1000*1000*1000});
			// cpu_timer::get_process().set_log_period(cpu_timer::CpuNs{0});
			cpu_timer::get_process().set_enabled(true);
		}

		void set_switchboard(std::shared_ptr<switchboard>&& sb_) {
			assert(!sb);
			sb = std::move(sb_);
			frame_data_writer = sb->get_writer<frame_data>("frames");
			sb->schedule<frame_data>(0, topic_name, [&](switchboard::ptr<const frame_data>&& frame_data, size_t) {
				switchboard_callback(
					frame_data->tid,
					frame_data->finished,
					frame_data->in_progress
				);
			});
		}

		void switchboard_callback(size_t tid, const cpu_timer::Frames& finished, const cpu_timer::Frames& in_progress) {
			std::vector<std::vector<sqlite::value>> frame_rows;
			std::vector<std::vector<sqlite::value>> string_rows;
			std::unordered_set<const char*> strings;

			for (const cpu_timer::Frames* frames : std::array<const cpu_timer::Frames*, 2>{&finished, &in_progress}) {
				for (const cpu_timer::Frame& frame : *frames) {
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
					});
					for (const char* string : std::array<const char*, 2>{frame.get_function_name(), frame.get_file_name()}) {
						if (strings.count(string) == 0) {
							strings.insert(string);
							string_rows.emplace_back(std::vector<sqlite::value>{
								sqlite::value{static_cast<uint64_t>(reinterpret_cast<intptr_t>(string))},
								sqlite::value{std::string{string}}
							});
						}
					}
				}
			}

			frame_table.bulk_insert(std::move(frame_rows));
			string_table.bulk_insert(std::move(string_rows));
		}
	};
}
