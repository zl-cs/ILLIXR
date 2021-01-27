#include <memory>
#include <functional>
#include <vector>

#include <boost/filesystem.hpp>
#include <sqlite3.h>

#include "common/switchboard.hpp"
#include "sqlite_wrapper.hpp"

namespace ILLIXR {

	template <typename Event>
	class sqlite_logger {
	private:
		std::shared_ptr<switchboard> sb;
		std::string topic_name;
		sqlite::schema schema;
		// In C++20, we can use ranges instead of vectors.
		// Vectors imply copying everything out of the switchboard event and into the vector.
		// Ranges would be lazily transforming them out of the switchboard event and into the SQLite calls.
		std::function<std::vector<std::vector<sqlite::value>>(switchboard::ptr<const Event>&)> to_records;
		sqlite::database database;
		sqlite::table table;
	public:
		sqlite_logger(
			std::shared_ptr<switchboard> sb_
			std::string&& topic_name_,
			sqlite::schema&& schema_,
			std::function<std::vector<std::vector<sqlite::value>>(switchboard::ptr<const Event>&)> to_records_,
			std::function<void()> destructor_,
		)
			: sb{sb_}
			, topic_name{std::move(topic_name_)}
			, schema{std::move(schema_)}
			, to_records{to_records_}
			, database{(boost::filesystem::path{"common"} / topic_name).replace_extension(".sqlite"), true}
			, table{database.create_table(topic_name, schema)}
		{
			sb->schedule<Event>(0, topic_name, [&](ptr<const Event>&& event, size_t) {
				database.begin_transaction();
				table.bulk_insert(logger.to_records(event));
				database.end_transaction();
			});
		}
	};

}
