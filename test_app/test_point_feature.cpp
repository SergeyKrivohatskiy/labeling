#include "test_point_feature.h"

using namespace geom2;
namespace labeling
{
    test_point_feature::test_point_feature(const geom2::point_i &position,
                                           const geom2::point_d &speed,
                                           const geom2::size_i &field_size,
                                           bool is_fixed,
                                           double /*rotation*/)
        :
          position(position),
          speed(speed),
          field_size(field_size),
          is_fixed(is_fixed),
          exact_position(position.x, position.y)
    {
        // TODO rotation
        label_size.h = 40;
        label_size.w = 100;
        label_offset.x = label_offset.y = 40;
        prefered_positions.push_back(prefered_position(1.0, label_offset));
        prefered_positions.push_back(prefered_position(1.0, point_i{-40, 40}));
        prefered_positions.push_back(prefered_position(0.3, point_i{40, -40}));
    }

    test_point_feature::~test_point_feature()
    {}

    const point_i& test_point_feature::get_screen_pivot() const
    {
        return position;
    }
    const size_i& test_point_feature::get_label_size() const
    {
        return label_size;
    }

    const point_i& test_point_feature::get_label_offset() const
    {
        return label_offset;
    }

    void test_point_feature::set_label_offset(const point_i &new_offset)
    {
        label_offset = new_offset;
    }

    const test_point_feature::prefered_pos_list&
                        test_point_feature::get_prefered_positions() const
    {
        return prefered_positions;
    }

    bool test_point_feature::is_label_fixed() const
    {
        return is_fixed;
    }

    void test_point_feature::set_fixed(bool fixed)
    {
        is_fixed = fixed;
    }

    void test_point_feature::update_position()
    {
        exact_position.x =
                fmod(exact_position.x + speed.x + field_size.w, field_size.w);
        exact_position.y =
                fmod(exact_position.y + speed.y + field_size.h, field_size.h);
        position.x = static_cast<int>(exact_position.x);
        position.y = static_cast<int>(exact_position.y);
    }

} // namespace labeling
