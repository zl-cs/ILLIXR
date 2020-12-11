#pragma once
#include <cstdlib>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <cstring>
#include <system_error>
#include <algorithm>
#include <unistd.h>
#include <memory>
#include <limits>
#include <deque>
#include <cassert>
#include <list>
#include <iostream>
#include <cerrno>
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
 *
 * TODO:
 * - Remove stack_frames that I successfully dump in ThreadContext::serialize(). This way, ThreadContext::serialize() (also ProcessContext::serialize() and GlobalState::serialize()) can be called multiple times
 * - Number invocations of the same function in the stack_frames. This way, args can be a comment. If it is ommitted, analysis can still precisely recreate the trace. It also then does not have to be duplicated in the parent record.
 */

#ifdef CPU_TIMER3_DISABLE
#define CPU_TIMER3_TIME_BLOCK()
#define CPU_TIMER3_TIME_BLOCK_(name, args)
#define CPU_TIMER3_THREAD_CONTEXT int*
#define CPU_TIMER3_MAKE_THREAD_CONTEXT() nullptr
#define CPU_TIMER3_SET_THREAD_CONTEXT(thread_context)
#define CPU_TIMER3_SERIALIZE()

namespace cpu_timer3 {
	class check_env_var {
		check_env_var() {
			const char* CPU_TIMER3_ENABLE = getenv("CPU_TIMER3_ENABLE");
			if (CPU_TIMER3_ENABLE && !!std::stoi(std::string{CPU_TIMER3_ENABLE})) {
				std::cerr <<
					"You set CPU_TIMER3_ENABLE=1 at runtime, but set CPU_TIMER3_DISABLE at compile-time.\n"
					"cpu_timer3 remains disabled.\n"
					"Recompile without CPU_TIMER3_DISABLE to fix.\n"
					;
			}
		}
	};
	static check_env_var check_env_var_instance;
}

#else // CPU_TIMER3_DISABLE

#define TOKENPASTE_(x, y) x ## y
#define TOKENPASTE(x, y) TOKENPASTE_(x, y)

#define bool_likely(x) x
#define bool_unlikely(x) x
#define FILESYSTEM_DEBUG

#define CPU_TIMER3_TIME_BLOCK() CPU_TIMER3_TIME_BLOCK_(__func__, "")
#define CPU_TIMER3_TIME_BLOCK_(name, args) auto TOKENPASTE(__cpu_timer3_timer, __LINE__) = \
		cpu_timer3::__state.should_profile() ? std::make_optional<cpu_timer3::StackFrameContext>(cpu_timer3::__state.get_current_thread_context(), name, args) : std::nullopt;
#define CPU_TIMER3_THREAD_CONTEXT cpu_timer3::ThreadContext*
#define CPU_TIMER3_MAKE_THREAD_CONTEXT() (cpu_timer3::__state.should_profile() ? cpu_timer3::__state.make_thread_context() : nullptr)
#define CPU_TIMER3_SET_THREAD_CONTEXT(thread_context) {if (cpu_timer3::__state.should_profile()) {cpu_timer3::__state.set_current_thread_context(*thread_context);}}
#define CPU_TIMER3_SERIALIZE() (cpu_timer3::__state.should_profile() ? std::make_optional<cpu_timer3::filesystem::path>(cpu_timer3::__state.serialize()) : std::nullopt)

namespace cpu_timer3 {

	using time_point = std::chrono::steady_clock::time_point;
	using seconds = std::chrono::seconds;
	using nanoseconds = std::chrono::nanoseconds;

	static time_point wall_now() {
		return std::chrono::steady_clock::now();
	}

	/**
	 * @brief A C++ translation of [clock_gettime][1]
	 *
	 * [1]: https://linux.die.net/man/3/clock_gettime
	 *
	 */
	static nanoseconds
	cpp_clock_gettime(clockid_t clock_id) {
		struct timespec ts;
		if (!clock_gettime(clock_id, &ts)) {
			std::cerr << "clock_gettime failed\n";
			abort();
		}
		return seconds{ts.tv_sec} + nanoseconds{ts.tv_nsec};
	}

	/**
	 * @brief Gets the CPU time for the calling thread.
	 */
	static nanoseconds
	cpu_now() {
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
		std::uniform_int_distribution<unsigned int> dist {0, 16};
		std::string ret (n, ' ');
		for (char& ch : ret) {
			unsigned int r = dist(rng);
			ch = (r < 10 ? '0' + r : 'a' + r - 10);
		}
		return ret;
	}

	/**
	 * @brief Replace all instances of @p from to @p to in @p str
	 *
	 * See https://stackoverflow.com/questions/3418231/replace-part-of-a-string-with-another-string
	 */
	static void replace_all(std::string& str, const std::string& from, const std::string& to) {
		if (from.empty()) {
			return;
		}
		size_t start_pos = 0;
		while((start_pos = str.find(from, start_pos)) != std::string::npos) {
			str.replace(start_pos, from.length(), to);
			start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
		}
	}

	static int fence() {
		std::atomic_thread_fence(std::memory_order_seq_cst);
		return 0;
	}

	template <typename Map, typename Vector>
	typename Map::mapped_type lookup(Map& map, Vector reverse_map, typename Map::key_type word) {
		auto it = map.find(word);
		if (it != map.end()) {
			assert(reverse_map[it->second] == it->first);
			return it->second;
		} else {
			auto val = map.size();
			map[word] = val;
			reverse_map.push_back(word);
			return val;
		}
	}

	class ThreadContext;

	/**
	 * @brief holds timing data relating to one stack-frame.
	 */
	class StackFrame {
	private:
		std::string comment;
		ThreadContext& thread;
		size_t frame_id;
		size_t function_id;
		const StackFrame* caller;
		bool _is_started;
		bool _is_stopped;
		int fence1;
		nanoseconds cpu_start;
		nanoseconds cpu_stop;
		time_point wall_start;
		time_point wall_stop;
		int fence2;
	public:
		StackFrame(std::string&& comment_, ThreadContext& thread_, size_t frame_id_, size_t function_id_, const StackFrame* caller_)
			: comment{std::move(comment_)}
			, thread{thread_}
			, frame_id{frame_id_}
			, function_id{function_id_}
			, caller{caller_}
			, _is_started{false}
			, _is_stopped{false}
		{ }
		void start_timer() {
			assert(!_is_started);
			assert(!_is_stopped);
			_is_started = true;
			fence1 = fence();
			cpu_stop = cpu_now();
			wall_stop = wall_now();
			fence2 = fence();
		}
		void stop_timer() {
			fence1 = fence();
			cpu_stop = cpu_now();
			wall_stop = wall_now();
			fence2 = fence();
			assert(_is_started);
			assert(!_is_stopped);
			_is_stopped = true;
		}
		bool is_timed() const { return _is_started && _is_stopped; }
		size_t get_function_id() const { return function_id; }
		void serialize(std::ostream&, const char*) const;
	};

	class ProcessContext;

	/**
	 * @brief A class that holds the stack for the current thread.
	 */
	class ThreadContext {
	private:
		size_t thread_id;
		time_point global_start_time;
		size_t unused_frame_id;
		std::deque<StackFrame> stack;
		std::deque<StackFrame> finished;
		std::unordered_map<const char*, size_t> function_name_to_id;
		std::vector<const char*> function_id_to_name;
	public:
		ThreadContext(size_t thread_id_, time_point global_start_time_)
			: thread_id{thread_id_}
			, global_start_time{global_start_time_}
			, unused_frame_id{1}
		{ }
		void enter_stack_frame(const char* function_name, std::string&& comment) {
			stack.emplace_back(
				std::move(comment),
				*this,
				unused_frame_id++,
				lookup(function_name_to_id, function_id_to_name, function_name),
				(stack.empty() ? nullptr : &stack.back()
			));

			// very last!
			stack.back().start_timer();
		}
		void exit_stack_frame([[maybe_unused]] const char* function_name) {
			// very first!
			stack.back().stop_timer();

			assert(function_name == function_id_to_name.at(stack.back().get_function_id()));
			finished.emplace_back(stack.back());
			stack.pop_back();
		}
		time_point get_start_time() { return global_start_time; }
		void serialize(std::ostream& os) {
			std::vector<bool> function_id_seen (function_id_to_name.size(), false);
			for (const StackFrame& stack_frame : finished) {
				assert(stack_frame.is_timed());
				size_t function_id = stack_frame.get_function_id();
				if (function_id_seen[function_id]) {
					stack_frame.serialize(os, "");
				} else {
					stack_frame.serialize(os, function_id_to_name[function_id]);
					function_id_seen[function_id] = true;
				}
			}
			finished.clear();
		}
		size_t get_thread_id() const { return thread_id; }
		ThreadContext(const ThreadContext&) = delete;
		ThreadContext& operator=(const ThreadContext&) = delete;
	};

	inline void StackFrame::serialize(std::ostream& os, const char* function_name) const {
			os
				<< thread.get_thread_id() << ","
				<< frame_id << ","
				<< function_id << ","
				<< (caller ? caller->frame_id : 0) << ","
				<< cpu_start.count() << ","
				<< (cpu_stop - cpu_start).count() << ","
				<< std::chrono::duration_cast<nanoseconds>(wall_start - thread.get_start_time()).count() << ","
				<< std::chrono::duration_cast<nanoseconds>(wall_stop - wall_start).count() << ","
				<< function_name << ","
				<< comment << "\n";
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
		time_point start_time;
		std::list<ThreadContext> threads;
	public:
		ProcessContext(time_point start_time_)
			: start_time{start_time_}
		{ }
		time_point get_start_time() { return start_time; }
		ThreadContext* make_thread() {
			threads.emplace_back(threads.size(), start_time);
			return &threads.back();
		}
		void serialize(std::ostream& os) {
			for (ThreadContext& thread : threads) {
				thread.serialize(os);
			}
		}
	};


	/**
	 * @brief An RAII context for creating, stopping, and storing StackFrames.
	 */
	class StackFrameContext {
	private:
		ThreadContext& thread;
		const char* site_name;
	public:
		StackFrameContext(const StackFrameContext&) = delete;
		StackFrameContext& operator=(const StackFrameContext&) = delete;
		StackFrameContext(ThreadContext& thread_, const char* site_name_, std::string&& comment)
			: thread{thread_}
			, site_name{site_name_}
		{
			thread.enter_stack_frame(site_name, std::move(comment));
		}
		~StackFrameContext() {
			thread.exit_stack_frame(site_name);
		}
	};

	/**
	 * A miniature port of a subset of the C++ filesystem stdlib specialized for POSIX.
	 *
	 * I take no joy in supporting antiquated systems.
	 */
	namespace filesystem {
		class path {
		public:
			typedef char value_type;
			typedef std::basic_string<value_type> string_type;
			path(const value_type* val_) : val{val_} { replace_all(val, "/", "\\/"); }
			path(string_type&& val_) : val{std::move(val_)} { replace_all(val, "/", "\\/"); }
			path(const string_type& val_) : val{std::move(val_)} { replace_all(val, "/", "\\/"); }
			path operator/(path other_path) const { return {val + "/" + other_path.val, true}; }
			path operator+(path other_path) const { return {val + other_path.val, true}; }
			const char* c_str() const { return val.c_str(); }
			std::string string() const { return val; }
		private:
			path(string_type&& val_, bool) : val{std::move(val_)} {}
			string_type val;
		};

		class filesystem_error : public std::system_error {
		public:
			filesystem_error(const std::string& what_arg, const path& p1, const path& p2, int ec)
				: filesystem_error(what_arg, p1, p2, std::make_error_code(std::errc(ec)))
			{ }
			filesystem_error(const std::string& what_arg, const path& p1, const path& p2, std::error_code ec)
				: system_error{ec, what_arg + " " + p1.string() + " " + p2.string() + ": " + std::to_string(ec.value())}
			{ }
			filesystem_error(const std::string& what_arg, const path& p1, std::error_code ec)
				: system_error{ec, what_arg + " " + p1.string() + ": " + std::to_string(ec.value())}
			{ }
			filesystem_error(const std::string& what_arg, const path& p1, int ec)
				: filesystem_error(what_arg, p1, std::make_error_code(std::errc(ec))) { }
		};

		class directory_iterator;

		class directory_entry {
		private:
			path this_path;
			bool _is_directory;
			bool _exists;
		public:
			directory_entry(const path& this_path_, std::error_code ec) : this_path{this_path_} { refresh(ec); }
			directory_entry(const path& this_path_) : this_path{this_path_} { refresh(); }
			void refresh(std::error_code& ec) {
				assert(errno == 0);
				struct stat buf;
				if (stat(this_path.c_str(), &buf)) {
					ec = std::make_error_code(std::errc(errno));
					errno = 0;
				} else {
					ec.clear();
					_is_directory = S_ISDIR(buf.st_mode);
					_exists = true;
				}
			}
			void refresh() {
				std::error_code ec;
				refresh(ec);
				if (ec) {
					if (ec.value() == ENOENT) {
						_is_directory = false;
						_exists = false;
					} else {
						throw filesystem_error{std::string{"stat"}, this_path, ec};
					}
				}
			}
			path path() const { return this_path; }
			bool is_directory() const { return _is_directory; }
			bool exists() { return _exists; }
		};

		class directory_iterator {
		private:
			std::deque<directory_entry> dir_entries;
		public:
			directory_iterator(const path& dir_path) {
				assert(errno == 0);
#ifdef FILESYSTEM_DEBUG
				std::cerr << "ls " << dir_path.string() << "\n";
#endif
				if (bool_likely(DIR* dir = opendir(dir_path.c_str()))) {
					assert(errno == 0);
					struct dirent* dir_entry = readdir(dir);
					while (bool_likely(dir_entry)) {
						bool skip = strcmp(dir_entry->d_name, ".") == 0 || strcmp(dir_entry->d_name, "..") == 0;
#ifdef FILESYSTEM_DEBUG
						std::cerr << "ls " << dir_path.string() << " -> " << dir_entry->d_name << " skip=" << skip << "\n";
#endif
						if (!skip) {
							dir_entries.emplace_back(path{dir_entry->d_name});
						}
						assert(errno == 0);
						dir_entry = readdir(dir);
					}
					if (bool_unlikely(errno != 0)) {
						int errno_ = errno; errno = 0;
						throw filesystem_error{std::string{"readdir"}, dir_path, errno_};
					}
					if (bool_unlikely(closedir(dir))) {
						int errno_ = errno; errno = 0;
						throw filesystem_error{std::string{"closedir"}, dir_path, errno_};
					}
				} else {
					int errno_ = errno; errno = 0;
					throw filesystem_error{std::string{"opendir"}, dir_path, errno_};
				}
			}
			directory_iterator() { }
			bool operator==(const directory_iterator& other) {
				if (dir_entries.size() == other.dir_entries.size()) {
					for (auto dir_entry0 = dir_entries.cbegin(), dir_entry1 = other.dir_entries.cbegin();
						 dir_entry0 != dir_entries.cend();
						 ++dir_entry0, ++dir_entry1) {
						if (dir_entry0 != dir_entry1) {
							return false;
						}
					}
					return true;
				} else {
					return false;
				}
			}
			bool operator!=(const directory_iterator& other) { return !(*this == other); }
			typedef directory_entry value_type;
			typedef std::ptrdiff_t difference_type;
			typedef const directory_entry* pointer;
			typedef const directory_entry& reference;
			typedef std::input_iterator_tag iterator_category;
			reference operator*() { assert(!dir_entries.empty()); return dir_entries.back(); }
			reference operator->() { return **this; }
			directory_iterator& operator++() {
				assert(!dir_entries.empty());
				dir_entries.pop_back();
				return *this;
			}
			directory_iterator operator++(int) {
				auto ret = *this;
				++*this;
				return ret;
			}
		};

		static std::deque<directory_entry> post_order(const path& this_path) {
			std::deque<directory_entry> ret;
			std::deque<directory_entry> stack;
			directory_entry this_dir_ent {this_path};
			if (this_dir_ent.exists()) {
				stack.push_back(this_dir_ent);
			}
			while (!stack.empty()) {
				directory_entry current {stack.front()};
				stack.pop_front();
				ret.push_front(current.path());
				if (current.is_directory()) {
					for (directory_iterator it {current.path()}; it != directory_iterator{}; ++it) {
						stack.push_back(*it);
					}
					// std::copy(directory_iterator{current.path()}, directory_iterator{}, stack.end() - 1);
				}
			}
			return ret;
		}

		static std::uintmax_t remove_all(const path& this_path) {
			assert(errno == 0);
			std::uintmax_t i = 0;
#ifdef FILESYSTEM_DEBUG
			std::cerr << "rm -rf " << this_path.string() << "\n";
#endif
			for (const directory_entry& descendent : post_order(this_path)) {
				++i;
				if (descendent.is_directory()) {
					assert(errno == 0);
#ifdef FILESYSTEM_DEBUG
					std::cerr << "rmdir " << descendent.path().string() << "\n";
#endif
					if (bool_unlikely(rmdir(descendent.path().c_str()))) {
						int errno_ = errno; errno = 0;
						throw filesystem_error(std::string{"rmdir"}, descendent.path(), errno_);
					}
				} else {
					assert(errno == 0);
#ifdef FILESYSTEM_DEBUG
					std::cerr << "unlink " << descendent.path().string() << "\n";
#endif
					if (bool_unlikely(unlink(descendent.path().c_str()))) {
						int errno_ = errno; errno = 0;
						throw filesystem_error(std::string{"unlink"}, descendent.path(), errno_);
					}
				}
			}
			return i;
		}

		static bool create_directory(const path& this_path) {
			std::error_code ec;
			directory_entry this_dir_ent{this_path, ec};
			if (!this_dir_ent.exists()) {
				mode_t umask_default = umask(0);
				umask(umask_default);
				int ret = mkdir(this_path.c_str(), 0777 & ~umask_default);
				if (ret == -1) {
					int errno_ = errno;
					errno = 0;
					throw filesystem_error(std::string{"mkdir"}, this_path, errno_);
				}
				return true;
			}
			return false;
		}

	}

	/*
	 * These global variables should ONLY be used by the macros.
	 *
	 * The rest of the code should not depend on global state.
	 */

	thread_local static ThreadContext* current_thread = nullptr;

	class GlobalState {
	private:
		bool should_profile_val;
		filesystem::path output_dir;
		time_point start_time;
		ProcessContext process;
		static time_point set_or_lookup_start_time([[maybe_unused]] const filesystem::path& output_dir) {
#ifdef OBVIOUSLY_FALSE
			filesystem::path start_time_path = output_dir / std::string{"start_time"};
			/*
			  Multiple object-files could be racing here, but it's ok,
			  because we round to the granularity of a second. If they
			  were racing, they should be relatively close in time.
			*/
			{
				std::ifstream start_time_ifile {start_time_path.string()};
				if (start_time_ifile.good()) {
					size_t start_time_int;
					start_time_ifile >> start_time_int;
					return time_point{seconds{start_time_int}};
				}
			} // close start_time_ifile here

			std::ofstream start_time_ofile {start_time_path.string()};
			size_t start_time_int = std::chrono::duration_cast<seconds>(wall_now().time_since_epoch()).count();
			start_time_ofile << start_time_int;
			return time_point{seconds{start_time_int}};
#else
			return time_point{seconds{0}};
#endif
		}
	public:
		GlobalState()
			: should_profile_val{!!std::stoi(getenv_or("CPU_TIMER3_ENABLE", "0"))}
			, output_dir{getenv_or("CPU_TIMER3_PATH", ".cpu_timer3")}
			, start_time{set_or_lookup_start_time(output_dir)}
			, process{start_time}
		{
			if (should_profile_val) {
				filesystem::remove_all(output_dir);
				filesystem::create_directory(output_dir);
			}
		}
		~GlobalState() {
			if (should_profile_val) {
				serialize();
			} else {
				std::cerr << "CPU_TIMER3 profiling is disabled; Set CPU_TIMER3_ENABLE=1 the env to enable.\n";
			}
		}
		filesystem::path serialize() {
			filesystem::path output_file_path {output_dir / random_hex_string() + std::string{".csv"}};
			std::ofstream output_file {output_file_path.string()};
			output_file
				<< "#{\"version\": \"3.2\", \"pandas_kwargs\": {\"dtype\": {\"comment\": \"str\"}, \"keep_default_na\": false, \"index_col\": [0, 1], \"comment\": \"#\"}}\n"
				<< "thread_id,frame_id,function_id,caller_frame_id,cpu_time_start,cpu_time,wall_time_start,wall_time,function_name,comment\n"
				;
			process.serialize(output_file);
			return output_file_path;
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
