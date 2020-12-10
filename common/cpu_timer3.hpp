#pragma once
#include <cstdlib>
#include <cstring>
// #include <experimental/filesystem>
#include <memory>
#include <limits>
#include <list>
#include <mutex>
#include <unordered_map>
#include <fstream>
#include <random>
#include <chrono>
#include <cstdint>
#include <optional>

/**
 * @brief This is the cpu_timer3.
 *
 * It is related to this: https://gist.github.com/charmoniumQ/975fda878ea9c4942a918440e94e433a
 *
 * - It is "opt-in" timing. By default, nothing is timed. Add the macro explicitly where you want timing.
 * - It uses RAII, so just have CPU_TIMER3_TIME_BLOCK() to time the block
 * - Each timing record maintains a record to the most recent caller that opted-in to timing. The timer is "context-sensitive".
 * - Each timing record can optionally contain arguments or runtime information.
 * - By default, the timer functionality is compiled in but turned off. Set CPU_TIMER3_DISABLE macro to not compile in, and set CPU_TIMER3_ENABLE env-var to turn on (if compiled in).
 * - Results in CSV get dumped at the program's shutdown time to the value of CPU_TIMER3_PATH env-var or `.cpu_timer3`.
 */

#ifdef CPU_TIMER3_DISABLE
#define CPU_TIMER3_TIME_BLOCK()
#define CPU_TIMER3_TIME_BLOCK_(name, args)
#define CPU_TIMER3_THREAD_CONTEXT int*
#define CPU_TIMER3_MAKE_THREAD_CONTEXT() nullptr
#define CPU_TIMER3_SET_THREAD_CONTEXT(thread_context)
#else // CPU_TIMER3_DISABLE

#define TOKENPASTE(x, y) x ## y
#define TOKENPASTE2(x, y) TOKENPASTE(x, y)

#define CPU_TIMER3_TIME_BLOCK() CPU_TIMER3_TIME_BLOCK_(__func__, "")
#define CPU_TIMER3_TIME_BLOCK_(name, args) auto TOKENPASTE(__cpu_timer3_timer, __LINE__) = \
		cpu_timer3::__state.should_profile() ? std::make_optional<cpu_timer3::StackFrameContext>(cpu_timer3::__state.get_current_thread_context(), name, args) : std::nullopt;
#define CPU_TIMER3_THREAD_CONTEXT cpu_timer3::ThreadContext*
#define CPU_TIMER3_MAKE_THREAD_CONTEXT() (cpu_timer3::__state.should_profile() ? cpu_timer3::__state.make_thread_context() : nullptr)
#define CPU_TIMER3_SET_THREAD_CONTEXT(thread_context) {if (cpu_timer3::__state.should_profile()) {cpu_timer3::__state.set_current_thread_context(*thread_context);}}

namespace cpu_timer3 {

	/**
	 * @brief A C++ translation of [clock_gettime][1]
	 *
	 * [1]: https://linux.die.net/man/3/clock_gettime
	 *
	 */
	static std::chrono::nanoseconds
	cpp_clock_gettime(clockid_t clock_id) {
		struct timespec ts;
		if (clock_gettime(clock_id, &ts)) {
			throw std::runtime_error{std::string{"clock_gettime returned "} + strerror(errno)};
		}
		return std::chrono::seconds{ts.tv_sec} + std::chrono::nanoseconds{ts.tv_nsec};
	}

	/**
	 * @brief Gets the CPU time for the calling thread.
	 */
	static inline std::chrono::nanoseconds
	thread_cpu_time() {
		return cpp_clock_gettime(CLOCK_THREAD_CPUTIME_ID);
	}

	/**
	 * @brief if var is env-var return it, else default_
	 */
	static std::string
	getenv_or(std::string var, std::string default_) {
		if (std::getenv(var.c_str())) {
			return {std::getenv(var.c_str())};
		} else {
			return default_;
		}
	}

	static std::string random_hex_string(size_t n = 16) {
		std::mt19937 rng {std::random_device{}()};
		std::uniform_int_distribution<unsigned int> dist {0, std::numeric_limits<unsigned int>::max()};
		std::string ret (n, ' ');
		static const std::string alphabet = "0123456789abcdef";
		for (char& ch : ret) {
			ch = alphabet[dist(rng) % alphabet.size()];
		}
		return ret;
	}

	/**
	 * @brief Replace all instances of @p from to @p to in @p str
	 *
	 * See https://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
	 */
	/*
	static void replaceAll(std::string& str, const std::string& from, const std::string& to) {
		if (from.empty()) {
			return;
		}
		size_t start_pos = 0;
		while((start_pos = str.find(from, start_pos)) != std::string::npos) {
			str.replace(start_pos, from.length(), to);
			start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
		}
	}
	*/

	static int fence() {
		std::atomic_thread_fence(std::memory_order_seq_cst);
		return 0;
	}

	class ThreadContext;

	/**
	 * @brief holds timing data relating to one stack-frame.
	 *
	 * Timer starts in constructor, and stops in stop()
	 */
	class StackFrame {
	private:
		const char* site_name;
		std::string args;
		const StackFrame* parent;
		ThreadContext& thread;
		int fence1;
		std::chrono::nanoseconds cpu_start;
		std::chrono::nanoseconds cpu_stop;
		std::chrono::steady_clock::time_point wall_start;
		std::chrono::steady_clock::time_point wall_stop;
		int fence2;
	public:
		StackFrame(const char* site_name_, std::string&& args_, const StackFrame* parent_, ThreadContext& thread_)
			: site_name{site_name_}
			, args{std::move(args_)}
			, parent{parent_}
			, thread{thread_}
			, fence1{fence()}
			, cpu_start{thread_cpu_time()}
			, wall_start{std::chrono::steady_clock::now()}
			, fence2{fence()}
		{ }
		void stop();
		void serialize(std::ostream& os, size_t thread_id, std::unordered_map<const char*, size_t>& map) const {
			if (map.count(site_name) == 0) {
				map[site_name] = map.size();
			}
			if (parent && map.count(parent->site_name) == 0) {
				map[parent->site_name] = map.size();
			}
			const char* thread = "thread";
			if (map.count(thread) == 0) {
				map[thread] = map.size();
			}
			os
				<< map[site_name] << ","
				<< args << ","
				<< map[(parent ? parent->site_name : thread)] << ","
				<< (parent ? parent->args : std::to_string(thread_id)) << ","
				<< cpu_start.count() << ","
				<< (cpu_stop - cpu_start).count() << ","
				<< std::chrono::duration_cast<std::chrono::nanoseconds>(wall_start.time_since_epoch()).count() << ","
				<< std::chrono::duration_cast<std::chrono::nanoseconds>(wall_stop - wall_start).count() << "\n";
		}
	};

	class ProcessContext;

	/**
	 * @brief A class that holds the stack for the current thread.
	 */
	class ThreadContext {
	private:
		const StackFrame* current;
		std::list<StackFrame> stack_frames;
		size_t thread_id;
		ProcessContext& process;
		size_t counter = 0;
	public:
		ThreadContext(size_t thread_id_, ProcessContext& process_)
			: current{nullptr}
			, thread_id{thread_id_}
			, process{process_}
		{ }
		void set_current(const StackFrame* new_current) { current = new_current; }
		StackFrame& make_stack_frame(const char* site_name, std::string&& args) {
			counter = (counter + 1) % 100;
			std::cout << counter << std::endl;
			if (counter == 0) {
				// TODO: remove this or make it more elegant
				// GlobalState should be the only one doing serialization
				subserialize();
			}
			stack_frames.emplace_back(site_name, std::move(args), current, *this);
			current = &stack_frames.back();
			return stack_frames.back();
		}
		void serialize(std::ostream& os, std::unordered_map<const char*, size_t>& map) {
			for (const StackFrame& stack_frame : stack_frames) {
				stack_frame.serialize(os, thread_id, map);
			}
			stack_frames.clear();
		}
		void subserialize();
		ThreadContext(const ThreadContext&) = delete;
		ThreadContext& operator=(const ThreadContext&) = delete;
	};

	inline void StackFrame::stop() {
		fence1 = fence();
		cpu_stop = thread_cpu_time();
		wall_stop = std::chrono::steady_clock::now();
		fence2 = fence();
		thread.set_current(parent);
	}

	/**
	 * @brief almost-process-level context
	 *
	 * This class holds holds threads, and it dumps them when it gets
	 * destructed.
	 *
	 * This is not necessarily process-level, if you have compiled and
	 * linked multiple object-files. Each object-file will have its
	 * own context.
	 */
	class ProcessContext {
	private:
		std::list<ThreadContext> threads;
		std::string output_path; // TODO: remove; Only GlobalState should care about this
	public:
		ProcessContext(std::string output_path_)
			: output_path{output_path_}
		{ }
		ThreadContext* make_thread() {
			threads.emplace_back(threads.size(), *this);
			return &threads.back();
		}
		void serialize(std::ostream& os, std::unordered_map<const char*, size_t>& map) {
			for (ThreadContext& thread : threads) {
				thread.serialize(os, map);
			}
		}
		std::string get_output_path() { return output_path; } // TODO: remove
	};

	
	inline void ThreadContext::subserialize() {
		// TODO: remove this or amke more elegant
			std::string name {random_hex_string()};
			std::string output_path = process.get_output_path();
			std::ofstream data_output {output_path + "/" + name + "_data.csv"};
			std::unordered_map<const char*, size_t> map;
			data_output << "# pandas.read_csv kwargs {\"dtype\": {\"args\": \"str\", \"parent_args\": \"str\"}, \"keep_default_na\": false}";
			data_output << "function,args,parent_function,parent_args,cpu_time_start,cpu_time,wall_time_start,wall_time\n";

			serialize(data_output, map);

			std::ofstream map_output {output_path + "/" + name + "_map.csv"};
			map_output << "function,function_name,\n";
			for (auto [word, code] : map) {
				map_output << word << "," << code << ",\n";
			}
		}

	/**
	 * @brief An RAII context for creating, stopping, and storing StackFrames.
	 */
	class StackFrameContext {
	private:
		StackFrame& this_frame;
	public:
		StackFrameContext(const StackFrameContext&) = delete;
		StackFrameContext& operator=(const StackFrameContext&) = delete;
		StackFrameContext(ThreadContext& thread, const char* site_name, std::string&& args)
			: this_frame{thread.make_stack_frame(site_name, std::move(args))}
		{ }
		~StackFrameContext() {
			this_frame.stop();
		}
	};

	/*
	 * These global variables should ONLY be used by the macros.
	 *
	 * The rest of the code should not depend on global state.
	 */

	thread_local static ThreadContext* current_thread = nullptr;

		// namespace fs = std::experimental::filesystem;

	class GlobalState {
	private:
		bool should_profile_val;
		// fs::path output_path;
		std::string output_path;
		ProcessContext process;
	public:
		GlobalState()
			: should_profile_val{!!std::stoi(getenv_or("CPU_TIMER3_ENABLE", "0"))}
			, output_path{getenv_or("CPU_TIMER3_PATH", ".cpu_timer3")}
			, process{output_path}
		{
			
			system(("rm -rf " + output_path).c_str());
			system(("mkdir " + output_path).c_str());
			// fs::remove_all(output_path);
			// fs::create_directory(output_path);
		}
		~GlobalState() {
			std::string name {random_hex_string()};
			std::ofstream data_output {output_path + "/" + name + "_data.csv"};
			std::unordered_map<const char*, size_t> map;
			data_output << "# pandas.read_csv kwargs {\"dtype\": {\"args\": \"str\", \"parent_args\": \"str\"}, \"keep_default_na\": false}";
			data_output << "function,args,parent_function,parent_args,cpu_time_start,cpu_time,wall_time_start,wall_time\n";
			process.serialize(data_output, map);

			std::ofstream map_output {output_path + "/" + name + "_map.csv"};
			map_output << "function,function_name,\n";
			for (auto [word, code] : map) {
				map_output << word << "," << code << ",\n";
			}
		}
		ThreadContext& get_current_thread_context() {
			return *(current_thread ? current_thread : (current_thread = make_thread_context()));
		}
		void set_current_thread_context(ThreadContext& other) { current_thread = &other; }
		ThreadContext* make_thread_context() { return process.make_thread(); }
		bool should_profile() { return should_profile_val; }
	};

	static GlobalState __state;

}

#endif // #else CPU_TIMER3_DISABLE
