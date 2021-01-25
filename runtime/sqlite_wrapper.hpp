#include <ostream>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <boost/filesystem.hpp>
#include <sqlite3.h>

#include "common/switchboard.hpp"

namespace ILLIXR {

	namespace sqlite {

		class statement_builder;
		class statement;

		void sqlite_error_to_exception(int rc, const char* activity) {
			if (rc != SQLITE_OK && rc != SQLITE_ROW && rc != SQLITE_DONE) {
				std::cerr << activity << ": " << sqlite3_errstr(rc);
				assert(0);
			}
		}

		class type {
		private:
			const char* name;
			std::string cpp_name;
		public:
			type(const char* name_, std::string&& cpp_name_)
				: name{name_}
				, cpp_name{cpp_name_}
			{ }

			type(const char* name_)
				: type{name_, name_}
			{ }

			const char* get_name() const { return name; }
			const std::string& get_cpp_name() const { return cpp_name; }

			bool operator==(const type& other) const {
				return this == &other;
			}

			static type NULL_;
			static type INTEGER;
			static type REAL;
			static type TEXT;
			static type BLOB;
		};
		type type::NULL_ = {"NULL", "NULL_"};
		type type::INTEGER = {"INTEGER"};
		type type::REAL = {"REAL"};
		type type::TEXT = {"TEXT"};
		type type::BLOB = {"BLOB"};

		std::ostream& operator<<(std::ostream& os, const type& type_) {
			return os << "type::" << type_.get_cpp_name();
		}

		class field {
		private:
			std::string _m_name;
			const type& _m_type;

		public:
			field(std::string&& name, const type& type)
				: _m_name{name}
				, _m_type{type}
			{ }

			const std::string& get_name() const { return _m_name; }
			const type& get_type() const { return _m_type; }
		};

		std::ostream& operator<<(std::ostream& os, const field& field_) {
			return os << "field{" << field_.get_name() << ", " << field_.get_type() << "}";
		}


		class schema {
		private:
			std::vector<field> _m_fields;

		public:
			schema(std::vector<field>&& fields)
				: _m_fields{std::move(fields)}
			{ }

			const std::vector<field>& get_fields() const { return _m_fields; }
		};

		std::ostream& operator<<(std::ostream& os, const schema& schema_) {
			os << "schema{{";
			bool first = true;
			for (const field& field : schema_.get_fields()) {
				if (!first) {
					os << ", ";
				}
				os << field;
				first = false;
			}
			return os << "}}";
		}

		class value {
		public:
			using value_variant = std::variant<uint64_t, double, std::string, std::vector<char>, std::nullptr_t, bool>;
		private:
			std::reference_wrapper<const type> _m_type;
			value_variant _m_data;
			size_t _m_id;
		public:
			value(const type& type_, value_variant&& data, size_t id = 0)
				: _m_type{std::cref(type_)}
				, _m_id{id}
			{
				set_data(std::move(data));
			}
				  
			static value placeholder(const type& type_, size_t id = 0) {
				return value{type_, false, id};
			}

			const type& get_type() const { return _m_type; }

			void set_id(size_t id) { assert(!has_id()); _m_id = id; }
			bool has_id() const { return _m_id == 0; }
			size_t get_id() const { assert(has_id()); return _m_id; }

			void set_data(value_variant&& data) {
				assert(!slot_filled());
				if (false) {
				} else if (_m_type.get() == type::INTEGER) {
					assert(std::holds_alternative<uint64_t>(data));
				} else if (_m_type.get() == type::REAL) {
					assert(std::holds_alternative<double>(data));
				} else if (_m_type.get() == type::TEXT) {
					assert(std::holds_alternative<std::string>(data));
				} else if (_m_type.get() == type::BLOB) {
					assert(std::holds_alternative<std::vector<char>>(data));
				} else if (_m_type.get() == type::NULL_) {
					assert(std::holds_alternative<std::nullptr_t>(data));
				} else {
					assert(0 && "Unkown type");
				}
				_m_data = std::move(data);
			}
			const value_variant& get_data() const {
				assert(has_data());
				return _m_data;
			}
			bool slot_filled() const { return !std::holds_alternative<bool>(_m_data) || std::get<bool>(_m_data); }
			bool has_data() const { return !std::holds_alternative<bool>(_m_data); }
			void mark_slot_filled() {
				assert(!slot_filled());
				_m_data = true;
			}
			void mark_slot_unfilled() {
				assert(slot_filled());
				_m_data = false;
			}
		};

		class database;
		class statement_builder;

		class statement {
		private:
			friend class statement_builder;
			std::reference_wrapper<database> _m_db;
			std::vector<value> _m_val_map;
			sqlite3_stmt* _m_stmt;

			statement(database& db, std::string cmd, std::vector<value> val_map);

			void bind(value var) {
				int rc = 0;
				if (false) {
				} else if (&var.get_type() == &type::INTEGER) {
					auto data = std::get<uint64_t>(var.get_data());
					rc = sqlite3_bind_int64(_m_stmt, var.get_id() + 1, data);
				} else if (&var.get_type() == &type::REAL) {
					auto data = std::get<double>(var.get_data());
					rc = sqlite3_bind_double(_m_stmt, var.get_id() + 1, data);
				} else if (&var.get_type() == &type::TEXT) {
					auto data = std::get<std::string>(var.get_data());
					rc = sqlite3_bind_text(_m_stmt, var.get_id() + 1, data.c_str(), data.size(), SQLITE_STATIC);
				} else if (&var.get_type() == &type::BLOB) {
					auto data = std::get<std::vector<char>>(var.get_data());
					rc = sqlite3_bind_blob(_m_stmt, var.get_id() + 1, data.data(), data.size(), SQLITE_STATIC);
				} else if (&var.get_type() == &type::NULL_) {
					rc = sqlite3_bind_null(_m_stmt, var.get_id() + 1);
				} else {
					assert(0 && "Unkown type");
				}
				sqlite_error_to_exception(rc, "bind");
			}

		public:
			void set(value&& var) {
				size_t id = var.get_id();
				assert(id < _m_val_map.size() + 1 && "Variable's integer ID out of range");
				value& target = _m_val_map.at(id);
				assert(!target.slot_filled() && "Variable's slot is already filled");
				assert(&target.get_type() == &var.get_type() && "Wrong type for variable");
				assert(var.has_data() && "Var must has value to use");
				_m_val_map.at(id) = std::move(var);
				bind(_m_val_map.at(id));
			}

			void execute() {
				for (const auto& pair : _m_val_map) {
					assert(pair.slot_filled());
				}
				int rc = sqlite3_step(_m_stmt);
				sqlite_error_to_exception(rc, "step_stmt");
			}

			void reset() {
				int rc = sqlite3_reset(_m_stmt);
				sqlite_error_to_exception(rc, "reset_stmt");
				for (auto& pair : _m_val_map) {
					pair.mark_slot_unfilled();
				}
			}

			~statement() {
				int rc = sqlite3_finalize(_m_stmt);
				sqlite_error_to_exception(rc, "finalize_stmt");
			}
		};

		class table;

		class database {
		private:
			friend class statement;
			friend class statement_builder;
			std::string _m_url;
			sqlite3* _m_db;
			bool in_transaction;
			std::optional<statement> begin_transaction_statement;
			std::optional<statement> end_transaction_statement;

			static std::string path_to_string(boost::filesystem::path&& path, bool truncate) {
				if (truncate) {
					if (boost::filesystem::exists(path)) {
						boost::filesystem::remove(path);
					}
				}
				return path.string();
			}

		public:
			database(std::string&& url)
				: _m_url{std::move(url)}
				, in_transaction{false}
			{
				int rc = sqlite3_open(url.c_str(), &_m_db);
				sqlite_error_to_exception(rc, "open database");
			}

			database(boost::filesystem::path&& path, bool truncate)
				: database{path_to_string(std::move(path), truncate)}
			{ }

			std::string get_url() const { return _m_url; }

			table create_table(std::string&& name, schema&& schema);

			~database() {
				int rc = sqlite3_close(_m_db);
				sqlite_error_to_exception(rc, "close");
			}

			// db will close the underlying db, so no copying (else it is closed twice).
			database(database& other) = delete;
			database& operator=(database& other) = delete;

			// table holds a ref to this, so no moving (else dangling ref).
			database(database&& other) = delete;
			database& operator=(database&& other) = delete;

			void begin_transaction();
			void end_transaction();
		};

		std::ostream& operator<<(std::ostream& os, const database& db) {
			return os << "database{\"" << db.get_url() << "\"}";
		}

		enum class sentinel {
			COMMA,
			LEFT_PARENS,
			RIGHT_PARENS
		};

		class statement_builder {
		private:
			database& _m_db;
			std::ostringstream _m_cmd_stream;
			size_t _m_first_unused_id;
			std::vector<std::optional<value>> _m_val_map;
			std::deque<bool> _m_comma_waiting;
			bool _m_valid;

		public:
			statement_builder(database& db, size_t num_vars)
				: _m_db{db}
				, _m_first_unused_id{0}
				, _m_val_map{num_vars}
				, _m_valid{false}
			{ }

			statement_builder& operator<<(const char* literal) {
				assert(_m_valid);
				if (!_m_comma_waiting.empty() && _m_comma_waiting.back()) {
					_m_cmd_stream << ", ";
				}
				_m_cmd_stream << literal << " ";
				return *this;
			}
			statement_builder& operator<<(value&& variable) {
				assert(_m_valid);
				if (!_m_comma_waiting.empty() && _m_comma_waiting.back()) {
					_m_cmd_stream << ", ";
				}
				assert(variable.get_id() + 1 < static_cast<size_t>(sqlite3_limit(_m_db._m_db, SQLITE_LIMIT_VARIABLE_NUMBER, -1)) && "Integer ID too high");
				if (variable.get_id() >= _m_val_map.size()) {
					_m_val_map.resize(variable.get_id() + 1);
				}
				if (!variable.has_id()) {
					variable.set_id(_m_first_unused_id++);
				}
				_m_cmd_stream << "?" << (variable.get_id() + 1) << " ";
				assert(!_m_val_map.at(variable.get_id()).has_value() || _m_val_map.at(variable.get_id())->get_type() == variable.get_type() &&
					   "Type mismatch with prior use of variable");
				assert(!_m_val_map.at(variable.get_id()).has_value() || _m_val_map.at(variable.get_id())->has_data() &&
					   "Cannot repeat the ID of a variable with a value");
				_m_val_map.at(variable.get_id()) = std::move(variable);
				return *this;
			}
			statement_builder& operator<<(sentinel sentinel) {
				assert(_m_valid);
				switch (sentinel) {
				case sentinel::COMMA:
					_m_comma_waiting.push_back(false);
					_m_cmd_stream << "( ";
					break;
				case sentinel::LEFT_PARENS:
					assert(!_m_comma_waiting.empty() && "More right than left parens");
					_m_comma_waiting.pop_back();
					_m_cmd_stream << ") ";
					break;
				case sentinel::RIGHT_PARENS:
					assert(!_m_comma_waiting.empty() && "comma only allowed inside parens");
					_m_comma_waiting.pop_back();
					_m_comma_waiting.push_back(true);
					break;
				}
				return *this;
			}
			statement compile();
		};

		class table {
		private:
			friend class database;
			std::reference_wrapper<database> _m_db;
			std::string _m_name;
			schema _m_schema;
			std::optional<statement> _m_insert_statement;

			table(database& db_, std::string&& name_, schema&& schema_)
				: _m_db{std::ref(db_)}
				, _m_name{std::move(name_)}
				, _m_schema{std::move(schema_)}
				, _m_insert_statement{std::nullopt}
			{ }

		public:

			const database& get_db() const { return _m_db; }

			const schema& get_schema() const { return _m_schema; }

			void bulk_insert(std::vector<std::vector<value>>&& rows) {
				// https://stackoverflow.com/questions/1711631/improve-insert-per-second-performance-of-sqlite

				if (!_m_insert_statement) {
					statement_builder sb {_m_db.get(), _m_schema.get_fields().size() + 1};
					sb << "INSERT INTO" << value{type::TEXT, _m_name} << "VALUES" << sentinel::LEFT_PARENS;
					for (const field& field_ : _m_schema.get_fields()) {
						sb << value::placeholder(field_.get_type()) << sentinel::COMMA;
					}
					sb << sentinel::RIGHT_PARENS;
					_m_insert_statement = sb.compile();
				}

				_m_db.get().begin_transaction();
				for (std::vector<value>& row : rows) {
					for (size_t i = 0; i < row.size(); ++i) {
						row.at(i).set_id(i);
						assert(row.at(i).get_type() == _m_schema.get_fields().at(i).get_type());
						_m_insert_statement->set(std::move(row.at(i)));
						_m_insert_statement->execute();
						_m_insert_statement->reset();
					}
				}
				_m_db.get().end_transaction();
			}

		};

		std::ostream& operator<<(std::ostream& os, const table& table) {
			return os << "table{" << table.get_db() << ", " << table.get_schema() << "}";
		}

		inline table database::create_table(std::string&& name, schema&& schema) {
				statement_builder statement_builder {*this, schema.get_fields().size() + 1};
				statement_builder << "CREATE TABLE" << value{type::TEXT, name} << sentinel::LEFT_PARENS;
				for (const field& field : schema.get_fields()) {
					statement_builder << value{type::TEXT, field.get_name()} << field.get_type().get_name() << sentinel::COMMA;
				}
				statement_builder << sentinel::RIGHT_PARENS;
				statement_builder.compile().execute();
				return table{*this, std::move(name), std::move(schema)};
			}

		inline statement statement_builder::compile() {
				assert(_m_comma_waiting.empty() && "More left parens than right parens");
				for (size_t i = 1; i < _m_val_map.size() + 1; ++i) {
					assert(_m_val_map.at(i).has_value() && "Not all integer IDs are used.");
				}
				_m_cmd_stream << ";";
				std::vector<value> val_map;
				val_map.reserve(_m_val_map.size());
				for (size_t i = 0; i < _m_val_map.size(); ++i) {
					val_map.push_back(std::move(*_m_val_map.at(i)));
				}
				_m_valid = false;
				return statement{_m_db, _m_cmd_stream.str(), val_map};
			}

		inline void database::begin_transaction() {
			assert(!in_transaction);
			in_transaction = true;
			if (!begin_transaction_statement) {
				begin_transaction_statement = (statement_builder{*this, 0} << "BEGIN TRANSACTION").compile();
			}
			begin_transaction_statement->execute();
		}
		inline void database::end_transaction() {
			assert(in_transaction);
			in_transaction = false;
			if (!end_transaction_statement) {
				end_transaction_statement = (statement_builder{*this, 0} << "END TRANSACTION").compile();
			}
			end_transaction_statement->execute();
		}

		inline statement::statement(database& db, std::string cmd, std::vector<value> val_map)
				: _m_db{std::ref(db)}
				, _m_val_map{val_map}
				, _m_stmt{nullptr}
			{
				int rc = sqlite3_prepare_v2(_m_db.get()._m_db, cmd.c_str(), cmd.size(), &_m_stmt, nullptr);
				sqlite_error_to_exception(rc, "prepare stmt");
				for (auto& value : _m_val_map) {
					if (value.has_data()) {
						set(std::move(value));
					}
				}
			}
	}
}
