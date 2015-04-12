#include "test_point_feature.h"

using namespace geom2;
namespace labeling
{
    test_point_feature::test_point_feature(const geom2::point_i &position,
                                           const geom2::point_i &speed,
                                           const geom2::size_i &field_size)
        :
          position(position),
          speed(speed),
          field_size(field_size)
    {
        label_size.h = 40;
        label_size.w = 100;
        label_offset.x = label_offset.y = 40;
        best_positions.push_back(label_offset);
        best_positions.push_back(point_i{-40, 40});
        good_positions.push_back(point_i{40, -40});
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

    const points_i_list& test_point_feature::labels_best_positions() const
    {
        return best_positions;
    }

    const points_i_list& test_point_feature::labels_good_positions() const
    {
        return good_positions;
    }

    void test_point_feature::update_position()
    {
        position.x = (position.x + speed.x + field_size.w) % field_size.w;
        position.y = (position.y + speed.y + field_size.h) % field_size.h;
    }

} // namespace labeling

