#include "data_format.hpp"
#include "phonebook.hpp"

using namespace ILLIXR;

class pose_prediction : public phonebook::service
{
public:
    [[nodiscard]] virtual auto get_fast_pose() const -> fast_pose_type                      = 0;
    [[nodiscard]] virtual auto get_true_pose() const -> pose_type                           = 0;
    [[nodiscard]] virtual auto get_fast_pose(time_type future_time) const -> fast_pose_type = 0;
    [[nodiscard]] virtual auto fast_pose_reliable() const -> bool                           = 0;
    [[nodiscard]] virtual auto true_pose_reliable() const -> bool                           = 0;
    virtual void               set_offset(const Eigen::Quaternionf& orientation)            = 0;
    virtual auto               get_offset() -> Eigen::Quaternionf                           = 0;
    [[nodiscard]] virtual auto correct_pose(pose_type pose) const -> pose_type              = 0;
    ~pose_prediction() override                                                             = default;
};
